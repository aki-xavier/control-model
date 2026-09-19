// body_tree.rs — BodyTree, a kinematic tree with a floating root: UrdfChain generalized to the
// biped's shape (trunk, two legs, a neck). A configuration is (base_p, base_q, q) and a velocity
// the base twist (omega, v) plus qd, so world-frame quantities come from FK. Parse scope: revolute
// joints plus one type="floating" root (or a rootless root link); fixed joints rejected.

use crate::mjcf_model::{MjcfJointExtra, MjcfModel, MjcfSite};
use crate::urdf::{parse_joint, parse_link, rpy_to_r, ChainJoint, ChainLink};
use crate::xml::parse_document;
use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use std::collections::HashMap;

/// TreeNode is one body of the tree: its parent joint's constant geometry plus its inertial record.
#[derive(Clone, Debug)]
pub struct TreeNode {
    pub name: String,
    /// node index of the parent body (-1 for the root)
    pub parent: i32,
    pub joint: String,
    /// this node's joint's index into q (-1 for the root)
    pub jo: i32,
    /// joint origin xyz and rotation in the parent frame, and the joint axis (unit) in the joint frame
    pub p_j: Vec3,
    pub r_j: Mat,
    pub axis: Vec3,
    pub mass: f64,
    pub com: Vec3,
    pub ic: Mat,
    pub damping: f64,
    pub friction: f64,
    pub limit_lo: f64,
    pub limit_hi: f64,
}

impl Default for TreeNode {
    fn default() -> Self {
        TreeNode {
            name: String::new(),
            parent: -1,
            joint: String::new(),
            jo: -1,
            p_j: Vec3::ZERO,
            r_j: Mat::zeros(3, 3),
            axis: Vec3::new(0.0, 0.0, 1.0),
            mass: 0.0,
            com: Vec3::ZERO,
            ic: Mat::zeros(3, 3),
            damping: 0.0,
            friction: 0.0,
            limit_lo: 0.0,
            limit_hi: 0.0,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct BodyTree {
    pub urdf: String,
    /// document order, parents before children
    pub nodes: Vec<TreeNode>,
    pub root: usize,
    /// actuated joints in q order (document order)
    pub q_names: Vec<String>,
    /// sidecar tables: task points, then armature/effort extras (both may be empty)
    pub sites: Vec<MjcfSite>,
    pub extras: Vec<MjcfJointExtra>,
}

impl BodyTree {
    /// n_q is the number of actuated joints; nv is 6 + n_q.
    pub fn n_q(&self) -> usize {
        self.q_names.len()
    }

    pub fn nv(&self) -> usize {
        6 + self.q_names.len()
    }

    pub fn q_index(&self, name: &str) -> Option<usize> {
        self.q_names.iter().position(|n| n == name)
    }

    pub fn node_of_q(&self, qj: usize) -> Option<usize> {
        self.nodes
            .iter()
            .position(|nd| nd.jo >= 0 && nd.jo as usize == qj)
    }

    pub fn node_index(&self, name: &str) -> Option<usize> {
        self.nodes.iter().position(|n| n.name == name)
    }

    /// contact_slot_of is the slot the engine's per-link CONTACT TABLE reports `name` under, or None
    /// for an unknown name. Slot `s` carries node `s + 1` and the floating root (node 0) rides the
    /// LAST slot, so a caller must say the link's NAME, never a slot number.
    pub fn contact_slot_of(&self, name: &str) -> Option<usize> {
        let ni = self.node_index(name)?;
        Some(if ni == 0 {
            self.nodes.len() - 1
        } else {
            ni - 1
        })
    }

    pub fn site(&self, name: &str) -> MjcfSite {
        for s in &self.sites {
            if s.name == name {
                return s.clone();
            }
        }
        MjcfSite {
            name: name.to_string(),
            ..Default::default()
        }
    }

    /// fk computes the world frames of all nodes: the root takes its absolute pose, then every child follows the chain recursion with its q slot.
    pub fn fk(&self, base_p: Vec3, base_q: Quat, q: &[f64]) -> (Vec<Vec3>, Vec<Mat>) {
        let n = self.nodes.len();
        let mut o = vec![Vec3::default(); n];
        let mut r = vec![Mat::zeros(3, 3); n];
        let mut t1 = Mat::zeros(3, 3);
        let mut t2 = Mat::zeros(3, 3);
        self.fk_into(base_p, base_q, q, &mut o, &mut r, (&mut t1, &mut t2));
        (o, r)
    }

    /// fk_into is fk into the caller's buffers, with two scratch matrices for the per-node intermediates.
    pub fn fk_into(
        &self,
        base_p: Vec3,
        base_q: Quat,
        q: &[f64],
        o: &mut Vec<Vec3>,
        r: &mut Vec<Mat>,
        scratch: (&mut Mat, &mut Mat),
    ) {
        let (t1, t2) = scratch;
        let n = self.nodes.len();
        o.clear();
        o.resize(n, Vec3::default());
        r.resize_with(n, || Mat::zeros(3, 3));
        for (i, nd) in self.nodes.iter().enumerate() {
            if nd.parent < 0 {
                o[i] = base_p;
                base_q.to_mat3_into(&mut r[i]);
            } else {
                let p = nd.parent as usize;
                let qi = if nd.jo >= 0 { q[nd.jo as usize] } else { 0.0 };
                r[p].mul_into(&nd.r_j, t1);
                Mat::from_axis_angle_into(nd.axis, qi, t2);
                t1.mul_into(t2, &mut r[i]);
                o[i] = o[p].add(r[p].mul_vec3(nd.p_j));
            }
        }
    }

    /// world_z: world axis of the node's parent joint = R_parent (r_j . axis).
    pub fn world_z(&self, r: &[Mat], i: usize) -> Vec3 {
        let nd = &self.nodes[i];
        if nd.jo < 0 {
            return Vec3::new(0.0, 0.0, 1.0);
        }
        let p = nd.parent as usize;
        r[p].mul_vec3(nd.r_j.mul_vec3(nd.axis)).normalized()
    }

    /// press_sign is the sign of the joint rotation that presses the node frame's <dir> side into the
    /// ground. It follows the joint's own world axis, so a mirrored limb cannot push the wrong way.
    pub fn press_sign(&self, r: &[Mat], qj: usize, dir: Vec3) -> f64 {
        match self.node_of_q(qj) {
            Some(ni) => {
                let a = self.world_z(r, ni);
                if a.cross(dir).z < 0.0 {
                    1.0
                } else {
                    -1.0
                }
            }
            None => 1.0,
        }
    }
    /// path_joints lists the q slots on the path from the root to node i, root-first.
    pub fn path_joints(&self, i: usize) -> Vec<usize> {
        let mut out: Vec<usize> = Vec::new();
        self.path_joints_into(i, &mut out);
        out
    }

    /// path_joints_into is path_joints into a caller's buffer (the mass matrix needs a path per node).
    pub fn path_joints_into(&self, i: usize, out: &mut Vec<usize>) {
        out.clear();
        let mut cur = i as i32;
        while cur >= 0 {
            let nd = &self.nodes[cur as usize];
            if nd.jo >= 0 {
                out.push(nd.jo as usize);
            }
            cur = nd.parent;
        }
        out.reverse();
    }

    pub fn link_com_w(&self, o: &[Vec3], r: &[Mat], i: usize) -> Vec3 {
        o[i].add(r[i].mul_vec3(self.nodes[i].com))
    }

    pub fn total_mass(&self) -> f64 {
        let mut s = 0.0;
        for nd in &self.nodes {
            s += nd.mass;
        }
        s
    }

    /// total_com_w: world CoM of the whole tree.
    pub fn total_com_w(&self, o: &[Vec3], r: &[Mat]) -> Vec3 {
        let mut c = Vec3::new(0.0, 0.0, 0.0);
        let mut m_tot = 0.0;
        for (i, nd) in self.nodes.iter().enumerate() {
            c = c.add(self.link_com_w(o, r, i).scale(nd.mass));
            m_tot += nd.mass;
        }
        c.scale(1.0 / m_tot)
    }

    /// com_jacobian: 3 x nv world CoM Jacobian. Base columns follow the world twist convention
    /// dc = omega x (c - o_root) + dv: angular block -skew(c - o_root), joint columns z_j x (c_i - o_j).
    pub fn com_jacobian(&self, o: &[Vec3], r: &[Mat]) -> Mat {
        let nv = self.nv();
        let mut j = Mat::zeros(3, nv);
        let m_tot = self.total_mass();
        let c = self.total_com_w(o, r);
        // the skew is written inline rather than built: `Mat::skew`'s 3 x 3 was an allocation a call
        let d = c.sub(o[self.root]);
        let sk = [[0.0, -d.z, d.y], [d.z, 0.0, -d.x], [-d.y, d.x, 0.0]];
        for rr in 0..3 {
            for cc in 0..3 {
                j.set(rr, cc, -sk[rr][cc]);
                j.set(rr, 3 + cc, if rr == cc { 1.0 } else { 0.0 });
            }
        }
        // ONE path buffer for the whole walk (examples/alloc_count_probe.rs).
        let mut path: Vec<usize> = Vec::new();
        for (i, nd) in self.nodes.iter().enumerate() {
            if nd.mass == 0.0 {
                continue;
            }
            let w = nd.mass / m_tot;
            let ci = self.link_com_w(o, r, i);
            self.path_joints_into(i, &mut path);
            for &qj in path.iter() {
                let ni = self.node_of_q(qj).expect("q slot has a node");
                let z = self.world_z(r, ni);
                let v = z.cross(ci.sub(o[ni]));
                for rr in 0..3 {
                    j.set(rr, 6 + qj, j.at(rr, 6 + qj) + w * v.to_array()[rr]);
                }
            }
        }
        j
    }

    /// link_spatial_jacobian: 6 x nv world spatial Jacobian of node i's frame, [angular; linear]; base angular identity + linear -skew(o_i - o_root), path joints z_j in both blocks.
    pub fn link_spatial_jacobian(&self, o: &[Vec3], r: &[Mat], i: usize) -> Mat {
        let nv = self.nv();
        let mut j = Mat::zeros(6, nv);
        let sk = Mat::skew(o[i].sub(o[self.root]));
        for rr in 0..3 {
            for cc in 0..3 {
                j.set(rr, cc, if rr == cc { 1.0 } else { 0.0 });
                j.set(3 + rr, 3 + cc, if rr == cc { 1.0 } else { 0.0 });
                j.set(3 + rr, cc, -sk.at(rr, cc));
            }
        }
        for qj in self.path_joints(i) {
            let ni = self.node_of_q(qj).expect("q slot has a node");
            let z = self.world_z(r, ni);
            let v = z.cross(o[i].sub(o[ni]));
            for rr in 0..3 {
                j.set(rr, 6 + qj, z.to_array()[rr]);
                j.set(3 + rr, 6 + qj, v.to_array()[rr]);
            }
        }
        j
    }

    /// link_spatial_com_jacobian: the same Jacobian for node i AT ITS CENTER OF MASS — the projection-form mass matrix integrates over this one.
    pub fn link_spatial_com_jacobian(&self, o: &[Vec3], r: &[Mat], i: usize) -> Mat {
        let mut j = Mat::zeros(6, self.nv());
        let mut path = Vec::new();
        self.link_spatial_com_jacobian_into(o, r, i, &mut j, &mut path);
        j
    }

    pub fn link_spatial_com_jacobian_into(
        &self,
        o: &[Vec3],
        r: &[Mat],
        i: usize,
        j: &mut Mat,
        path: &mut Vec<usize>,
    ) {
        let nv = self.nv();
        if j.rows != 6 || j.cols != nv {
            *j = Mat::zeros(6, nv);
        } else {
            for v in j.data.iter_mut() {
                *v = 0.0;
            }
        }
        let ci = self.link_com_w(o, r, i);
        // skew(ci - o[root]) INLINE, in Mat::skew's own layout: the angular block below is minus it.
        let d = ci.sub(o[self.root]);
        let sk = [[0.0, -d.z, d.y], [d.z, 0.0, -d.x], [-d.y, d.x, 0.0]];
        for rr in 0..3 {
            for cc in 0..3 {
                j.set(rr, cc, if rr == cc { 1.0 } else { 0.0 });
                j.set(3 + rr, 3 + cc, if rr == cc { 1.0 } else { 0.0 });
                j.set(3 + rr, cc, -sk[rr][cc]);
            }
        }
        self.path_joints_into(i, path);
        for &qj in path.iter() {
            let ni = self.node_of_q(qj).expect("q slot has a node");
            let z = self.world_z(r, ni);
            let v = z.cross(ci.sub(o[ni]));
            for rr in 0..3 {
                j.set(rr, 6 + qj, z.to_array()[rr]);
                j.set(3 + rr, 6 + qj, v.to_array()[rr]);
            }
        }
    }

    /// site_frame: world pose of a named site (its body's frame composed with its constant offset); zero pose for an unknown name.
    pub fn site_frame(&self, o: &[Vec3], r: &[Mat], name: &str) -> (Vec3, Mat) {
        for s in &self.sites {
            if s.name == name {
                match self.node_index(&s.body) {
                    Some(i) => {
                        let ri = r[i].mul(&s.quat.to_mat3());
                        let oi = o[i].add(r[i].mul_vec3(s.pos));
                        return (oi, ri);
                    }
                    None => return (Vec3::ZERO, control_math::mat::Mat::eye(3)),
                }
            }
        }
        (Vec3::ZERO, control_math::mat::Mat::eye(3))
    }

    /// offset_point_jacobian is the 3 x nv linear Jacobian of a point rigidly held by node i at `off`
    /// in that node's LOCAL frame — the counterpart of site_point_jacobian for an unnamed point.
    pub fn offset_point_jacobian(&self, o: &[Vec3], r: &[Mat], i: usize, off: Vec3) -> Mat {
        let mut j = Mat::zeros(3, self.nv());
        let mut path = Vec::new();
        self.offset_point_jacobian_into(o, r, i, off, &mut j, &mut path);
        j
    }

    /// offset_point_jacobian_into is the same into the caller's buffers, with the 3 x 3 skew inlined.
    pub fn offset_point_jacobian_into(
        &self,
        o: &[Vec3],
        r: &[Mat],
        i: usize,
        off: Vec3,
        j: &mut Mat,
        path: &mut Vec<usize>,
    ) {
        let nv = self.nv();
        if j.rows != 3 || j.cols != nv {
            *j = Mat::zeros(3, nv);
        } else {
            for v in j.data.iter_mut() {
                *v = 0.0;
            }
        }
        if i == 0 || i >= self.nodes.len() {
            return;
        }
        let sp = o[i].add(r[i].mul_vec3(off));
        let d = sp.sub(o[self.root]);
        let sk = [[0.0, -d.z, d.y], [d.z, 0.0, -d.x], [-d.y, d.x, 0.0]];
        for rr in 0..3 {
            for cc in 0..3 {
                j.set(rr, cc, -sk[rr][cc]);
                j.set(rr, 3 + cc, if rr == cc { 1.0 } else { 0.0 });
            }
        }
        self.path_joints_into(i, path);
        for &qj in path.iter() {
            let ni = self.node_of_q(qj).expect("q slot has a node");
            let z = self.world_z(r, ni);
            let v = z.cross(sp.sub(o[ni]));
            for rr in 0..3 {
                j.set(rr, 6 + qj, v.to_array()[rr]);
            }
        }
    }

    /// site_point_jacobian: 3 x nv linear Jacobian of a site point.
    pub fn site_point_jacobian(&self, o: &[Vec3], r: &[Mat], name: &str) -> Mat {
        let nv = self.nv();
        let (sp, _) = self.site_frame(o, r, name);
        let mut j = Mat::zeros(3, nv);
        let sk = Mat::skew(sp.sub(o[self.root]));
        for rr in 0..3 {
            for cc in 0..3 {
                j.set(rr, cc, -sk.at(rr, cc));
                j.set(rr, 3 + cc, if rr == cc { 1.0 } else { 0.0 });
            }
        }
        let Some(i) = self.node_index(&self.site(name).body) else {
            return j;
        };
        for qj in self.path_joints(i) {
            let ni = self.node_of_q(qj).expect("q slot has a node");
            let z = self.world_z(r, ni);
            let v = z.cross(sp.sub(o[ni]));
            for rr in 0..3 {
                j.set(rr, 6 + qj, v.to_array()[rr]);
            }
        }
        j
    }
}

/// copy_frames copies FK frames into a caller's buffers, reusing the matrices' own storage (a `Mat` is a `Vec<f64>`).
pub fn copy_frames(o: &[Vec3], r: &[Mat], o_out: &mut Vec<Vec3>, r_out: &mut Vec<Mat>) {
    o_out.clear();
    o_out.extend_from_slice(o);
    r_out.resize_with(r.len(), || Mat::zeros(3, 3));
    for (dst, src) in r_out.iter_mut().zip(r.iter()) {
        dst.copy_from(src);
    }
}

/// load_body_tree parses a URDF with exactly one floating root joint into a BodyTree; links and
/// joints reuse urdf.rs's element parsers. The q order is the ENGINE's canonical order, not the
/// document's (children sorted alphanumerically by link name, DFS pre-order), so state and torque
/// vectors pass verbatim; MJCF-ordered data is re-mapped by joint name (MjcfModel::keyframe_q).
pub fn load_body_tree(urdf_path: &str, meta: &MjcfModel) -> Result<BodyTree, String> {
    let src = std::fs::read_to_string(urdf_path)
        .map_err(|e| format!("simu.body_tree: cannot read {urdf_path}: {e}"))?;
    let root = parse_document(&src)?;
    let mut joints: Vec<ChainJoint> = Vec::new();
    let mut links: HashMap<String, ChainLink> = HashMap::new();
    let mut link_names: Vec<String> = Vec::new();
    for el in &root.children {
        if el.name == "joint" {
            joints.push(parse_joint(el));
        } else if el.name == "link" {
            let ln = parse_link(el);
            links.insert(ln.name.clone(), ln.clone());
            link_names.push(ln.name);
        }
    }
    let mut t = BodyTree {
        urdf: urdf_path.to_string(),
        sites: meta.sites.clone(),
        extras: meta.joints.clone(),
        ..Default::default()
    };
    // the floating root: an explicit type="floating" joint (its child is the root body), else the
    // unique link that is no joint's child (the engine's own convention)
    let mut root_link = String::new();
    for j in &joints {
        if j.jtype == "floating" {
            if !root_link.is_empty() {
                return Err("simu.body_tree: multiple floating joints".to_string());
            }
            root_link = j.child.clone();
        }
    }
    if root_link.is_empty() {
        let mut is_child: HashMap<String, bool> = HashMap::new();
        for j in &joints {
            is_child.insert(j.child.clone(), true);
        }
        for name in &link_names {
            if !is_child.contains_key(name) {
                if !root_link.is_empty() {
                    return Err("simu.body_tree: multiple root links".to_string());
                }
                root_link = name.clone();
            }
        }
    }
    if root_link.is_empty() {
        return Err("simu.body_tree: no root link found".to_string());
    }
    let mut node_idx: HashMap<String, usize> = HashMap::new();
    let rl = links
        .get(&root_link)
        .cloned()
        .ok_or_else(|| format!("simu.body_tree: root link {root_link} missing"))?;
    t.nodes.push(TreeNode {
        name: root_link.clone(),
        parent: -1,
        jo: -1,
        mass: rl.mass,
        com: rl.com,
        ic: link_inertia_world(&rl),
        ..Default::default()
    });
    node_idx.insert(root_link.clone(), 0);
    t.root = 0;
    // per-parent joint lists, then DFS pre-order with children sorted by CHILD LINK name — the engine's canonical order
    let mut by_parent: HashMap<String, Vec<ChainJoint>> = HashMap::new();
    for j in &joints {
        if j.jtype == "floating" {
            continue;
        }
        if j.jtype != "revolute" {
            return Err(format!(
                "simu.body_tree: joint {} of type {} unsupported",
                j.name, j.jtype
            ));
        }
        by_parent
            .entry(j.parent.clone())
            .or_default()
            .push(j.clone());
    }
    for parent in &link_names {
        if !by_parent.contains_key(parent) {
            continue;
        }
        // the sort key is the child attribute; a missing link fails later with a message, not a 0 here
        if let Some(kids) = by_parent.get_mut(parent) {
            kids.sort_by(|a, b| a.child.cmp(&b.child));
        }
    }
    // DFS pre-order: children pushed in reverse so the alphanumerically first pops first
    let mut stack: Vec<ChainJoint> = Vec::new();
    let root_kids = by_parent.get(&root_link).cloned().unwrap_or_default();
    for k in (0..root_kids.len()).rev() {
        stack.push(root_kids[k].clone());
    }
    while let Some(j) = stack.pop() {
        let pi = *node_idx
            .get(&j.parent)
            .ok_or_else(|| format!("simu.body_tree: joint {}: parent link missing", j.name))?;
        let cl = links.get(&j.child).cloned().ok_or_else(|| {
            format!(
                "simu.body_tree: joint {}: child link {} missing",
                j.name, j.child
            )
        })?;
        let jo = t.q_names.len() as i32;
        t.q_names.push(j.name.clone());
        let mut ax = j.axis;
        if ax.norm() < 1e-12 {
            ax = Vec3::new(1.0, 0.0, 0.0);
        }
        node_idx.insert(j.child.clone(), t.nodes.len());
        t.nodes.push(TreeNode {
            name: j.child.clone(),
            parent: pi as i32,
            joint: j.name.clone(),
            jo,
            p_j: j.xyz,
            r_j: rpy_to_r(&j.rpy),
            axis: ax.normalized(),
            mass: cl.mass,
            com: cl.com,
            ic: link_inertia_world(&cl),
            damping: j.damping,
            friction: 0.0,
            limit_lo: j.lower,
            limit_hi: j.upper,
        });
        let kids = by_parent.get(&j.child).cloned().unwrap_or_default();
        for k in (0..kids.len()).rev() {
            stack.push(kids[k].clone());
        }
    }
    // fold the sidecar's friction into the nodes (URDF carries damping, MJCF frictionloss the meta)
    for nd in t.nodes.iter_mut() {
        if nd.jo >= 0 {
            let ex = meta.joint_extra(&nd.joint);
            nd.friction = ex.friction;
        }
    }
    Ok(t)
}

/// link_inertia_world rotates a parsed link's inertia into the link frame (the inertial rpy is in
/// the URDF record).
fn link_inertia_world(lk: &ChainLink) -> Mat {
    let rin = rpy_to_r(&lk.inertial_rpy);
    rin.mul(&lk.inertia).mul(&rin.transposed())
}
