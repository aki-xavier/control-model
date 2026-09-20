// urdf.rs — parsed serial-chain model from URDF: per-joint origin/axis/damping, per-child-link
// inertial parameters, and the chain ordering from base link to end link. Only all-revolute serial
// chains are supported, with fixed joints allowed only at the END (they fold into the terminal tip
// offset): a fixed joint mid-chain is rejected rather than folded, since folding it would silently
// drop a joint from the q vector.

use crate::pga_layer::rotor_from_mat;
use crate::xml::parse_document;
use crate::xml::XmlNode;
use control_base::plant::motor_of_rotor;
use control_math::mat::Mat;
use control_math::vec3::Vec3;
use pga::Multivector;
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct ChainJoint {
    pub name: String,
    pub jtype: String,
    pub parent: String,
    pub child: String,
    pub xyz: Vec3,
    pub rpy: Vec3,
    pub axis: Vec3,
    pub damping: f64,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug)]
pub struct ChainLink {
    pub name: String,
    pub mass: f64,
    pub com: Vec3,
    pub inertial_rpy: Vec3,
    pub inertia: Mat,
}

#[derive(Clone, Debug)]
pub struct UrdfChain {
    pub joint_names: Vec<String>,
    pub child_names: Vec<String>,
    pub n: usize,
    pub p_j: Vec<Vec3>,
    pub r_j: Vec<Mat>,
    pub axis: Vec<Vec3>,
    pub dampings: Vec<f64>,
    pub link_mass: Vec<f64>,
    pub link_com: Vec<Vec3>,
    pub link_ic: Vec<Mat>,
    pub limit_lo: Vec<f64>,
    pub limit_hi: Vec<f64>,
    /// terminal tip offset: zero/identity when the chain ends at a revolute link.
    pub tip_p: Vec3,
    pub tip_r: Mat,
    pub tool_reach: f64,
}

pub fn load_urdf_chain(
    urdf_path: &str,
    base_link: &str,
    end_link: &str,
) -> Result<UrdfChain, String> {
    let src = std::fs::read_to_string(urdf_path)
        .map_err(|e| format!("simu.urdf: cannot read {urdf_path}: {e}"))?;
    let robot = parse_document(&src)?;
    let mut joints: Vec<ChainJoint> = Vec::new();
    let mut links: HashMap<String, ChainLink> = HashMap::new();
    for el in &robot.children {
        if el.name == "joint" {
            joints.push(parse_joint(el));
        } else if el.name == "link" {
            let ln = parse_link(el);
            links.insert(ln.name.clone(), ln);
        }
    }

    // walk BACKWARD from endLink: unambiguous for branching models, and it is what puts the
    // trailing fixed joints where they can fold into tip_p/tip_r, in the terminal revolute frame.
    let mut by_child: HashMap<String, ChainJoint> = HashMap::new();
    for j in joints {
        by_child.insert(j.child.clone(), j);
    }
    let mut rev: Vec<ChainJoint> = Vec::new();
    let mut cur = end_link.to_string();
    for _ in 0..=by_child.len() {
        let j = match by_child.get(&cur) {
            Some(j) => j.clone(),
            None => break,
        };
        rev.push(j.clone());
        cur = j.parent.clone();
        if cur == base_link {
            break;
        }
    }
    if rev.is_empty() || rev[rev.len() - 1].parent != base_link {
        return Err(format!(
            "simu.urdf_chain: no serial chain from {base_link} to {end_link}"
        ));
    }
    let mut tip_p = Vec3::new(0.0, 0.0, 0.0);
    let mut tip_r = Mat::eye(3);
    while !rev.is_empty() && rev[0].jtype == "fixed" {
        let tj = rev.remove(0);
        let tjr = rpy_to_r(&tj.rpy);
        tip_p = tj.xyz.add(tjr.mul_vec3(tip_p));
        tip_r = tjr.mul(&tip_r);
    }
    if rev.is_empty() {
        return Err(format!(
            "simu.urdf_chain: no revolute joints from {base_link} to {end_link}"
        ));
    }
    for j in &rev {
        if j.jtype != "revolute" {
            return Err(
                "simu.urdf_chain: only revolute joints are supported inside a serial chain"
                    .to_string(),
            );
        }
    }
    rev.reverse();
    let chain = rev;

    let mut m = UrdfChain {
        joint_names: Vec::new(),
        child_names: Vec::new(),
        n: chain.len(),
        p_j: Vec::new(),
        r_j: Vec::new(),
        axis: Vec::new(),
        dampings: Vec::new(),
        link_mass: Vec::new(),
        link_com: Vec::new(),
        link_ic: Vec::new(),
        limit_lo: Vec::new(),
        limit_hi: Vec::new(),
        tip_p,
        tip_r,
        tool_reach: 0.051,
    };
    for j in &chain {
        m.joint_names.push(j.name.clone());
        m.child_names.push(j.child.clone());
        m.p_j.push(j.xyz);
        let r = rpy_to_r(&j.rpy);
        m.r_j.push(r);
        m.axis.push(j.axis.normalized());
        m.dampings.push(j.damping);
        let lk = links.get(&j.child).cloned().unwrap_or_else(|| ChainLink {
            name: j.child.clone(),
            mass: 0.0,
            com: Vec3::ZERO,
            inertial_rpy: Vec3::ZERO,
            inertia: Mat::zeros(3, 3),
        });
        m.link_mass.push(lk.mass);
        m.link_com.push(lk.com);
        let rin = rpy_to_r(&lk.inertial_rpy);
        m.link_ic.push(rin.mul(&lk.inertia).mul(&rin.transposed()));
        m.limit_lo.push(j.lower);
        m.limit_hi.push(j.upper);
    }
    Ok(m)
}

pub(crate) fn parse_joint(el: &XmlNode) -> ChainJoint {
    let mut j = ChainJoint {
        name: el.attr_or("name", ""),
        jtype: el.attr_or("type", ""),
        axis: Vec3::new(0.0, 0.0, 1.0),
        lower: -1e308,
        upper: 1e308,
        ..Default::default()
    };
    for c in &el.children {
        match c.name.as_str() {
            "origin" => {
                j.xyz = parse_vec3(&c.attr_or("xyz", ""));
                j.rpy = parse_vec3(&c.attr_or("rpy", ""));
            }
            "axis" => {
                j.axis = parse_vec3(&c.attr_or("xyz", ""));
            }
            "parent" => {
                j.parent = c.attr_or("link", "");
            }
            "child" => {
                j.child = c.attr_or("link", "");
            }
            "dynamics" => {
                let d = c.attr_or("damping", "");
                if !d.is_empty() {
                    j.damping = f64_attr(&d);
                }
            }
            "limit" => {
                let lo = c.attr_or("lower", "");
                let hi = c.attr_or("upper", "");
                if !lo.is_empty() {
                    j.lower = f64_attr(&lo);
                }
                if !hi.is_empty() {
                    j.upper = f64_attr(&hi);
                }
            }
            _ => {}
        }
    }
    j
}

pub(crate) fn parse_link(el: &XmlNode) -> ChainLink {
    let mut l = ChainLink {
        name: el.attr_or("name", ""),
        mass: 0.0,
        com: Vec3::ZERO,
        inertial_rpy: Vec3::ZERO,
        inertia: Mat::zeros(3, 3),
    };
    for c in &el.children {
        if c.name != "inertial" {
            continue;
        }
        for in_ in &c.children {
            match in_.name.as_str() {
                "origin" => {
                    l.com = parse_vec3(&in_.attr_or("xyz", ""));
                    l.inertial_rpy = parse_vec3(&in_.attr_or("rpy", ""));
                }
                "mass" => {
                    l.mass = f64_attr(&in_.attr_or("value", ""));
                }
                "inertia" => {
                    l.inertia.set(0, 0, f64_attr(&in_.attr_or("ixx", "0")));
                    l.inertia.set(1, 1, f64_attr(&in_.attr_or("iyy", "0")));
                    l.inertia.set(2, 2, f64_attr(&in_.attr_or("izz", "0")));
                    l.inertia.set(0, 1, f64_attr(&in_.attr_or("ixy", "0")));
                    l.inertia.set(0, 2, f64_attr(&in_.attr_or("ixz", "0")));
                    l.inertia.set(1, 2, f64_attr(&in_.attr_or("iyz", "0")));
                    let i01 = l.inertia.at(0, 1);
                    let i02 = l.inertia.at(0, 2);
                    let i12 = l.inertia.at(1, 2);
                    l.inertia.set(1, 0, i01);
                    l.inertia.set(2, 0, i02);
                    l.inertia.set(2, 1, i12);
                }
                _ => {}
            }
        }
    }
    l
}

/// parse_vec3 splits an attribute on single spaces; a doubled separator leaves that component at
/// zero rather than shifting the rest of the vector.
fn parse_vec3(s: &str) -> Vec3 {
    let parts: Vec<&str> = s.split(' ').collect();
    let mut v = Vec3::ZERO;
    if !parts.is_empty() && !parts[0].is_empty() {
        v.x = f64_attr(parts[0]);
    }
    if parts.len() > 1 && !parts[1].is_empty() {
        v.y = f64_attr(parts[1]);
    }
    if parts.len() > 2 && !parts[2].is_empty() {
        v.z = f64_attr(parts[2]);
    }
    v
}

/// f64_attr parses a float attribute: a value it cannot read is 0.0, not an error, since URDF
/// leaves attributes out freely. Public because it is also read across the crate boundary.
pub fn f64_attr(s: &str) -> f64 {
    s.trim().parse::<f64>().unwrap_or(0.0)
}

/// rpy_to_r: URDF's own convention — fixed-axis RPY, i.e. extrinsic XYZ, so R = Rz Ry Rx.
pub fn rpy_to_r(rpy: &Vec3) -> Mat {
    let r = rpy.x;
    let p = rpy.y;
    let y = rpy.z;
    let mut rx = Mat::zeros(3, 3);
    rx.set(0, 0, 1.0);
    rx.set(1, 1, r.cos());
    rx.set(1, 2, -r.sin());
    rx.set(2, 1, r.sin());
    rx.set(2, 2, r.cos());
    let mut ry = Mat::zeros(3, 3);
    ry.set(0, 0, p.cos());
    ry.set(0, 2, p.sin());
    ry.set(1, 1, 1.0);
    ry.set(2, 0, -p.sin());
    ry.set(2, 2, p.cos());
    let mut rz = Mat::zeros(3, 3);
    rz.set(0, 0, y.cos());
    rz.set(0, 1, -y.sin());
    rz.set(1, 0, y.sin());
    rz.set(1, 1, y.cos());
    rz.set(2, 2, 1.0);
    rz.mul(&ry).mul(&rx)
}

pub fn urdf_path() -> String {
    crate::models::z1_urdf().to_string_lossy().to_string()
}

/// home_q is the task start configuration (deg2rad of [20 70 -85 35 0 17]).
pub fn home_q() -> Vec<f64> {
    vec![
        0.3490658503988659,
        1.2217304763960306,
        -1.4835298641951802,
        0.6108652381980153,
        0.0,
        0.29670597283903605,
    ]
}

impl UrdfChain {
    /// fk returns the per-link world frames as `(origins, rotations)`, in chain order.
    pub fn fk(&self, q: &[f64]) -> (Vec<Vec3>, Vec<Mat>) {
        let n = self.n;
        let mut o: Vec<Vec3> = Vec::with_capacity(n);
        let mut r: Vec<Mat> = Vec::with_capacity(n);
        let mut o_prev = Vec3::new(0.0, 0.0, 0.0);
        let mut r_prev = Mat::eye(3);
        for i in 0..n {
            let ri = r_prev
                .mul(&self.r_j[i])
                .mul(&Mat::from_axis_angle(self.axis[i], q[i]));
            let oi = o_prev.add(r_prev.mul_vec3(self.p_j[i]));
            o.push(oi);
            r.push(ri.clone());
            o_prev = oi;
            r_prev = ri;
        }
        (o, r)
    }

    pub fn world_z(&self, r: &[Mat], k: usize) -> Vec3 {
        let mut z = self.r_j[k].mul_vec3(self.axis[k]);
        if k > 0 {
            z = r[k - 1].mul_vec3(z);
        }
        z.normalized()
    }

    /// The task reference point's world position and its rotation MATRIX — the terminal frame plus the tip
    /// offset and the tool reach along the terminal local +x, orientation including the tip rotation.
    ///
    /// Why the pair is computed once and read twice below: the quaternion reading and the rotor reading are
    /// two spellings of one pose, and a second copy of this arithmetic is where the two would stop agreeing.
    fn tip_frame(&self, o: &[Vec3], r: &[Mat]) -> (Vec3, Mat) {
        let i = o.len() - 1;
        let off = self.tip_p.add(Vec3::new(self.tool_reach, 0.0, 0.0));
        (o[i].add(r[i].mul_vec3(off)), r[i].mul(&self.tip_r))
    }

    /// tip_position: the task reference point's world position on its own, and the reading the callers that
    /// only want the point use. Why it is not `motor_position` on `tip_motor` below: that is a pose built and
    /// taken apart, which is not the identity in floating point, so the point would come back moved in its
    /// last bits for asking a smaller question.
    pub fn tip_position(&self, o: &[Vec3], r: &[Mat]) -> Vec3 {
        self.tip_frame(o, r).0
    }

    /// tip_motor: the same task reference point as one PGA motor, for the callers whose pose vocabulary is the
    /// algebra's — the position and the rotation are the same two values `tip_position` and the rotation read,
    /// not a round trip through a quaternion and back.
    pub fn tip_motor(&self, o: &[Vec3], r: &[Mat]) -> Multivector {
        let (p, rot) = self.tip_frame(o, r);
        motor_of_rotor(p, rotor_from_mat(&rot))
    }

    /// point_jacobian: 3 x n world linear Jacobian of a world point attached to the chain, all joints active.
    pub fn point_jacobian(&self, o: &[Vec3], r: &[Mat], p: Vec3) -> Mat {
        let n = self.n;
        let mut j = Mat::zeros(3, n);
        for c in 0..n {
            let v = self.world_z(r, c).cross(p.sub(o[c]));
            j.set(0, c, v.x);
            j.set(1, c, v.y);
            j.set(2, c, v.z);
        }
        j
    }

    /// link_jacobian is point_jacobian's convention at link frame i: columns zero past joint i.
    pub fn link_jacobian(&self, o: &[Vec3], r: &[Mat], i: usize) -> Mat {
        let n = self.n;
        let mut j = Mat::zeros(3, n);
        for c in 0..=i {
            let v = self.world_z(r, c).cross(o[i].sub(o[c]));
            j.set(0, c, v.x);
            j.set(1, c, v.y);
            j.set(2, c, v.z);
        }
        j
    }

    /// full_jacobian: 6 x n at a world point attached to the terminal link, stacked [linear; angular].
    pub fn full_jacobian(&self, o: &[Vec3], r: &[Mat], p: Vec3) -> Mat {
        let n = self.n;
        let mut j = Mat::zeros(6, n);
        for c in 0..n {
            let z = self.world_z(r, c);
            let v = z.cross(p.sub(o[c]));
            j.set(0, c, v.x);
            j.set(1, c, v.y);
            j.set(2, c, v.z);
            j.set(3, c, z.x);
            j.set(4, c, z.y);
            j.set(5, c, z.z);
        }
        j
    }
}
