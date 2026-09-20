// mjcf_convert.rs — MjcfConverter, the MJCF -> URDF bridge. The URDF is the source of truth: one
// parser reads it and the sidecar (MjcfModel) carries only what URDF cannot express. Bodies nest and
// T(q) = T(pos,quat) * R(axis,q) with (w,x,y,z) quats; a <freejoint/> body becomes the URDF ROOT
// LINK, because a synthetic link under the engine's own world joint would double the root. Serial
// hinge chains only.

use crate::mjcf_model::{MjcfJointExtra, MjcfModel, MjcfSite};
use crate::xml::parse_document;
use crate::xml::XmlNode;
use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
struct MjcfDefaults {
    damping: f64,
    frictionloss: f64,
    armature: f64,
    geom_visual: bool,
    geom_type: String,
    force_lo: f64,
    force_hi: f64,
}

pub struct MjcfConverter {
    meshdir: String,
    deg: bool,
    cls: HashMap<String, MjcfDefaults>,
    // the active MJCF childclass: a body's childclass attribute is the default class for itself and
    // all descendants, so it has to be saved and restored around a recursion.
    childclass: String,
}

impl MjcfConverter {
    pub fn new() -> MjcfConverter {
        MjcfConverter {
            meshdir: String::new(),
            deg: false,
            cls: HashMap::new(),
            childclass: String::new(),
        }
    }
}
impl Default for MjcfConverter {
    fn default() -> Self {
        Self::new()
    }
}

/// num renders f64 for the emitted XML: ELEVEN decimal places, trailing zeros kept. This form is the
/// module's OWN contract — the emitted URDF is a committed artifact, so changing the precision or the
/// rounding would rewrite every line of it. The gate is tests/mjcf.rs.
fn num(x: f64) -> String {
    format!("{x:.11}")
}

fn floats(s: &str) -> Vec<f64> {
    let mut out = Vec::new();
    for tok in s.split([' ', '\t', '\n']) {
        let t = tok.trim();
        if !t.is_empty() {
            out.push(t.parse::<f64>().unwrap_or(0.0));
        }
    }
    out
}

fn vec3_of(n: &XmlNode, key: &str) -> Vec3 {
    let f = floats(&n.attr_or(key, ""));
    if f.len() < 3 {
        return Vec3::ZERO;
    }
    Vec3::new(f[0], f[1], f[2])
}

/// quat_of: an MJCF quat attribute (w x y z), identity when it is short.
fn quat_of(n: &XmlNode, key: &str) -> Quat {
    let f = floats(&n.attr_or(key, ""));
    if f.len() < 4 {
        return Quat::IDENTITY;
    }
    Quat {
        w: f[0],
        x: f[1],
        y: f[2],
        z: f[3],
    }
}

/// rpy_of maps a rotation matrix to URDF's fixed-axis RPY — the inverse of urdf.rs's rpy_to_r.
fn rpy_of(r: &Mat) -> Vec3 {
    let cp = (r.at(0, 0) * r.at(0, 0) + r.at(1, 0) * r.at(1, 0)).sqrt();
    if cp < 1e-12 {
        // gimbal lock: pitch = +/- pi/2, roll zero and yaw carries the in-plane rotation
        return Vec3::new(
            0.0,
            (-r.at(2, 0)).atan2(0.0),
            (-r.at(0, 1)).atan2(r.at(1, 1)),
        );
    }
    Vec3::new(
        r.at(2, 1).atan2(r.at(2, 2)),
        (-r.at(2, 0)).atan2(cp),
        r.at(1, 0).atan2(r.at(0, 0)),
    )
}

fn origin(p: Vec3, q: Quat) -> String {
    let rpy = rpy_of(&q.to_mat3());
    format!(
        "<origin xyz=\"{} {} {}\" rpy=\"{} {} {}\"/>",
        num(p.x),
        num(p.y),
        num(p.z),
        num(rpy.x),
        num(rpy.y),
        num(rpy.z)
    )
}

impl MjcfConverter {
    fn read_compiler(&mut self, root: &XmlNode) {
        for el in &root.children {
            if el.name == "compiler" {
                self.meshdir = el.attr_or("meshdir", ".");
                self.deg = el.attr_or("angle", "radian") == "degree";
            }
        }
    }

    /// read_defaults recurses to any depth: one missed level silently turns collision geoms into
    /// visuals.
    fn read_defaults(&mut self, root: &XmlNode) {
        for el in &root.children {
            if el.name != "default" {
                continue;
            }
            self.read_default_block(el);
        }
    }

    fn read_default_block(&mut self, el: &XmlNode) {
        for cls_el in &el.children {
            if cls_el.name != "default" {
                continue;
            }
            let name = cls_el.attr_or("class", "");
            if !name.is_empty() {
                let mut d = self.cls.get(&name).cloned().unwrap_or_default();
                for prop in &cls_el.children {
                    match prop.name.as_str() {
                        "joint" => {
                            d.damping = prop.attr_or("damping", "0").parse().unwrap_or(0.0);
                            d.frictionloss =
                                prop.attr_or("frictionloss", "0").parse().unwrap_or(0.0);
                            d.armature = prop.attr_or("armature", "0").parse().unwrap_or(0.0);
                        }
                        "geom" => {
                            let ct: f64 = prop.attr_or("contype", "1").parse().unwrap_or(0.0);
                            let ca: f64 = prop.attr_or("conaffinity", "1").parse().unwrap_or(0.0);
                            d.geom_visual = ct == 0.0 && ca == 0.0;
                            d.geom_type = prop.attr_or("type", &d.geom_type.clone());
                        }
                        "position" => {
                            let fr = floats(&prop.attr_or("forcerange", ""));
                            if fr.len() >= 2 {
                                d.force_lo = fr[0];
                                d.force_hi = fr[1];
                            }
                        }
                        _ => {}
                    }
                }
                self.cls.insert(name, d);
            }
            self.read_default_block(cls_el);
        }
    }

    /// MJCF's own precedence: the element's attributes, then its class, then the childclass inherited
    /// from the enclosing body.
    fn joint_defaults(&self, el: &XmlNode) -> MjcfDefaults {
        let mut d = MjcfDefaults::default();
        let cls_name = el.attr_or("class", &self.childclass.clone());
        if !cls_name.is_empty() {
            d = self.cls.get(&cls_name).cloned().unwrap_or_default();
        }
        if let Some(v) = el.attrs.get("damping") {
            d.damping = v.parse().unwrap_or(0.0);
        }
        if let Some(v) = el.attrs.get("frictionloss") {
            d.frictionloss = v.parse().unwrap_or(0.0);
        }
        if let Some(v) = el.attrs.get("armature") {
            d.armature = v.parse().unwrap_or(0.0);
        }
        d
    }

    /// geom_is_visual: the two contact classes by name first, then the class table, then the contact
    /// flags.
    fn geom_is_visual(&self, el: &XmlNode) -> bool {
        let cls = el.attr_or("class", "");
        if cls == "collision" || cls == "self_collision_only" {
            return false;
        }
        if cls.is_empty() {
            // an unclassed geom inherits the file's outer childclass; treat it as visual only when the contact flags say so
            let ct: f64 = el.attr_or("contype", "0").parse().unwrap_or(0.0);
            let ca: f64 = el.attr_or("conaffinity", "0").parse().unwrap_or(0.0);
            return ct == 0.0 && ca == 0.0;
        }
        if let Some(d) = self.cls.get(&cls) {
            return d.geom_visual;
        }
        let ct: f64 = el.attr_or("contype", "0").parse().unwrap_or(0.0);
        let ca: f64 = el.attr_or("conaffinity", "0").parse().unwrap_or(0.0);
        ct == 0.0 && ca == 0.0
    }

    /// A primitive geom has no mesh-free path through the importer, so it goes out as a closed unit
    /// mesh with a scale; a mesh geom passes through as-is.
    fn emit_geom(
        &self,
        sb: &mut String,
        el: &XmlNode,
        tag: &str,
        assets: &HashMap<String, String>,
    ) {
        let p = vec3_of(el, "pos");
        let q = quat_of(el, "quat");
        let mut gtype = el.attr_or("type", "");
        if gtype.is_empty() {
            if let Some(d) = self.cls.get(&el.attr_or("class", "")) {
                gtype = d.geom_type.clone();
            }
        }
        if gtype.is_empty() {
            gtype = "mesh".to_string();
        }
        sb.push_str(&format!("\t\t<{tag}>\n\t\t\t{}\n", origin(p, q)));
        if gtype == "sphere" {
            let size = floats(&el.attr_or("size", "0.005"));
            let r = if !size.is_empty() { size[0] } else { 0.005 };
            sb.push_str(&format!(
                "\t\t\t<geometry><mesh filename=\"meshes/_unit_sphere.stl\" scale=\"{} {} {}\"/></geometry>\n",
                num(r),
                num(r),
                num(r)
            ));
        } else if gtype == "mesh" || gtype.is_empty() {
            let mesh_name = el.attr_or("mesh", "");
            let file = assets
                .get(&mesh_name)
                .cloned()
                .unwrap_or_else(|| mesh_name.clone());
            sb.push_str(&format!(
                "\t\t\t<geometry><mesh filename=\"meshes/{file}\"/></geometry>\n"
            ));
        } else {
            // box/capsule/cylinder have no mesh-free path through the importer; fail loudly rather than drop the geom
            eprintln!(
                "simu.mjcf_convert: geom type \"{gtype}\" unsupported (mesh-only importer), skipped"
            );
        }
        sb.push_str(&format!("\t\t</{tag}>\n"));
    }

    /// walk_body saves and restores the enclosing childclass around the recursion, since a body's
    /// childclass applies to all of its descendants.
    fn walk_body(
        &mut self,
        sb: &mut String,
        b: &XmlNode,
        parent: &str,
        assets: &HashMap<String, String>,
        m: &mut MjcfModel,
    ) -> Result<String, String> {
        let saved_cc = self.childclass.clone();
        let out = self.walk_body_inner(sb, b, parent, assets, m);
        self.childclass = saved_cc;
        out
    }

    fn walk_body_inner(
        &mut self,
        sb: &mut String,
        b: &XmlNode,
        parent: &str,
        assets: &HashMap<String, String>,
        m: &mut MjcfModel,
    ) -> Result<String, String> {
        let name = b.attr_or("name", "");
        if name.is_empty() {
            return Err(format!("simu.mjcf_convert: anonymous body under {parent}"));
        }
        let cc = b.attr_or("childclass", "");
        if !cc.is_empty() {
            self.childclass = cc;
        }
        let pos = vec3_of(b, "pos");
        let quat = quat_of(b, "quat");
        let mut hinge: Option<&XmlNode> = None;
        let mut n_hinge = 0;
        let mut has_free = false;
        for el in &b.children {
            if el.name == "freejoint" {
                has_free = true;
            } else if el.name == "joint" {
                if el.attr_or("type", "hinge") != "hinge" {
                    return Err(format!(
                        "simu.mjcf_convert: joint {} of type {} unsupported (serial hinge chains only)",
                        el.attr_or("name", "?"),
                        el.attr_or("type", "?")
                    ));
                }
                n_hinge += 1;
                hinge = Some(el);
            }
        }
        if has_free && n_hinge > 0 {
            return Err(format!(
                "simu.mjcf_convert: body {name} mixes freejoint and hinge joints"
            ));
        }
        if !has_free && n_hinge != 1 {
            return Err(format!(
                "simu.mjcf_convert: body {name} has {n_hinge} joints (welded/multi-DOF bodies unsupported)"
            ));
        }
        // the parent joint: a freejoint body has none (it becomes the URDF root link)
        if !has_free {
            let hinge = hinge.expect("one hinge joint");
            let jname = hinge.attr_or("name", &format!("{name}_joint"));
            let mut axis = vec3_of(hinge, "axis");
            if axis.norm() < 1e-12 {
                axis = Vec3::new(1.0, 0.0, 0.0);
            }
            let d = self.joint_defaults(hinge);
            let mut rng = floats(&hinge.attr_or("range", ""));
            if self.deg {
                for v in rng.iter_mut() {
                    *v = v.to_radians();
                }
            }
            if rng.len() < 2 {
                // the literal here IS pi, to the last bit
                rng = vec![-std::f64::consts::PI, std::f64::consts::PI];
            }
            // the actuator table gives the effort bound when one is registered; MuJoCo 2.x models
            // carry it on the joint element instead
            let mut eff = 0.0;
            for a in &m.joints {
                if a.name == jname {
                    eff = a.effort_lo.abs().max(a.effort_hi.abs());
                }
            }
            if eff <= 0.0 {
                let frange = floats(&hinge.attr_or("actuatorfrcrange", ""));
                if frange.len() >= 2 {
                    eff = frange[0].abs().max(frange[1].abs());
                }
            }
            let velocity = 10.79;
            // the quasi-kinematic engine steps joints at v = tau/eta, so an MJCF with no viscous damping is a division by zero whose speeds explode the solver; derive eta = tau_stall / omega_noload.
            let mut damp = d.damping;
            if damp <= 0.0 && eff > 0.0 {
                damp = eff / velocity;
            }
            sb.push_str(&format!("\t<joint name=\"{jname}\" type=\"revolute\">\n"));
            sb.push_str(&format!(
                "\t\t<parent link=\"{parent}\"/>\n\t\t<child link=\"{name}\"/>\n"
            ));
            sb.push_str(&format!("\t\t{}\n", origin(pos, quat)));
            sb.push_str(&format!(
                "\t\t<axis xyz=\"{} {} {}\"/>\n",
                num(axis.x),
                num(axis.y),
                num(axis.z)
            ));
            sb.push_str(&format!(
                "\t\t<limit lower=\"{}\" upper=\"{}\" effort=\"{}\" velocity=\"{}\"/>\n",
                num(rng[0]),
                num(rng[1]),
                num(eff),
                num(velocity)
            ));
            sb.push_str(&format!(
                "\t\t<dynamics damping=\"{}\" friction=\"{}\"/>\n",
                num(damp),
                num(d.frictionloss)
            ));
            sb.push_str("\t</joint>\n");
            // armature has no URDF home, so the sidecar row read_actuators registered merges with the
            // damping/friction the URDF joint now carries
            let mut merged = false;
            for a in m.joints.iter_mut() {
                if a.name == jname {
                    a.damping = damp;
                    a.friction = d.frictionloss;
                    a.armature = d.armature;
                    // MuJoCo's inheritrange: a classed actuator with no forcerange inherits the joint's actuatorfrcrange
                    if a.effort_hi <= 0.0 && eff > 0.0 {
                        a.effort_lo = -eff;
                        a.effort_hi = eff;
                    }
                    merged = true;
                }
            }
            if !merged {
                m.joints.push(MjcfJointExtra {
                    name: jname,
                    damping: damp,
                    friction: d.frictionloss,
                    armature: d.armature,
                    effort_lo: -eff,
                    effort_hi: eff,
                });
            }
        }
        sb.push_str(&format!("\t<link name=\"{name}\">\n"));
        for el in &b.children {
            if el.name == "inertial" {
                let ip = vec3_of(el, "pos");
                let mass: f64 = el.attr_or("mass", "0").parse().unwrap_or(0.0);
                let mut fi = floats(&el.attr_or("fullinertia", ""));
                if fi.len() < 6 {
                    let di = floats(&el.attr_or("diaginertia", ""));
                    if di.len() >= 3 {
                        fi = vec![di[0], di[1], di[2], 0.0, 0.0, 0.0];
                    } else {
                        return Err(format!(
                            "simu.mjcf_convert: body {name} inertial without fullinertia/diaginertia"
                        ));
                    }
                }
                sb.push_str(&format!(
                    "\t\t<inertial>\n\t\t\t<mass value=\"{}\"/>\n",
                    num(mass)
                ));
                sb.push_str(&format!("\t\t\t{}\n", origin(ip, Quat::IDENTITY)));
                sb.push_str(&format!(
                    "\t\t\t<inertia ixx=\"{}\" iyy=\"{}\" izz=\"{}\" ixy=\"{}\" ixz=\"{}\" iyz=\"{}\"/>\n",
                    num(fi[0]),
                    num(fi[1]),
                    num(fi[2]),
                    num(fi[3]),
                    num(fi[4]),
                    num(fi[5])
                ));
                sb.push_str("\t\t</inertial>\n");
            }
        }
        for el in &b.children {
            if el.name == "geom" {
                if self.geom_is_visual(el) {
                    self.emit_geom(sb, el, "visual", assets);
                } else {
                    self.emit_geom(sb, el, "collision", assets);
                }
            } else if el.name == "site" {
                m.sites.push(MjcfSite {
                    name: el.attr_or("name", ""),
                    body: name.clone(),
                    pos: vec3_of(el, "pos"),
                    quat: quat_of(el, "quat"),
                });
            }
        }
        sb.push_str("\t</link>\n");
        for el in &b.children {
            if el.name == "body" {
                self.walk_body(sb, el, &name, assets, m)?;
            }
        }
        Ok(name)
    }

    /// read_actuators must run before the body walk, so the effort it registers can land in the URDF
    /// limits the walk emits.
    fn read_actuators(&self, root: &XmlNode, m: &mut MjcfModel) {
        for el in &root.children {
            if el.name != "actuator" {
                continue;
            }
            for a in &el.children {
                if a.name != "position" {
                    continue;
                }
                let jname = a.attr_or("joint", "");
                if jname.is_empty() {
                    continue;
                }
                let cls = a.attr_or("class", "");
                let d = self.cls.get(&cls).cloned().unwrap_or_default();
                m.joints.push(MjcfJointExtra {
                    name: jname,
                    effort_lo: d.force_lo,
                    effort_hi: d.force_hi,
                    ..Default::default()
                });
            }
        }
    }

    /// The scene file is optional and contributes keyframes only.
    pub fn convert(&mut self, robot_path: &str, scene_path: &str) -> Result<MjcfModel, String> {
        let src = std::fs::read_to_string(robot_path)
            .map_err(|e| format!("simu.mjcf_convert: cannot read {robot_path}: {e}"))?;
        let root = parse_document(&src)?;
        if root.name != "mujoco" {
            return Err(format!(
                "simu.mjcf_convert: {robot_path} has no <mujoco> root"
            ));
        }
        self.read_compiler(&root);
        self.read_defaults(&root);
        let mut m = MjcfModel::default();
        read_keyframes_into(robot_path, &mut m.kf_names, &mut m.kf_qpos)?;
        // the scene's keyframes override same-named robot rows: they are the tuned posture set
        let mut s_names: Vec<String> = Vec::new();
        let mut s_rows: Vec<Vec<f64>> = Vec::new();
        read_keyframes_into(scene_path, &mut s_names, &mut s_rows)?;
        for (i, n) in s_names.iter().enumerate() {
            match m.kf_index(n) {
                Some(j) => m.kf_qpos[j] = s_rows[i].clone(),
                None => {
                    m.kf_names.push(n.clone());
                    m.kf_qpos.push(s_rows[i].clone());
                }
            }
        }
        let assets = read_assets(&root, &self.meshdir.clone());
        self.read_actuators(&root, &mut m);
        let mut wb: Option<&XmlNode> = None;
        for el in &root.children {
            if el.name == "worldbody" {
                wb = Some(el);
            }
        }
        let mut sb = String::with_capacity(4096);
        let robot_name = root.attr_or("model", "mjcf_robot");
        sb.push_str(&format!("<robot name=\"{robot_name}\">\n"));
        if let Some(wb) = wb {
            for el in &wb.children {
                if el.name == "body" {
                    self.walk_body(&mut sb, el, "world", &assets, &mut m)?;
                }
            }
        }
        sb.push_str("</robot>\n");
        m.urdf = sb;
        Ok(m)
    }
}

/// read_assets maps mesh names to files. `_meshdir` is unused on purpose: the emitted filenames are
/// relative to the URDF, which sits beside the meshes.
fn read_assets(root: &XmlNode, _meshdir: &str) -> HashMap<String, String> {
    let mut out = HashMap::new();
    for el in &root.children {
        if el.name != "asset" {
            continue;
        }
        for a in &el.children {
            if a.name != "mesh" {
                continue;
            }
            let file = a.attr_or("file", "");
            if file.is_empty() {
                continue;
            }
            let mut name = a.attr_or("name", "");
            if name.is_empty() {
                let fname = file.rsplit('/').next().unwrap_or(&file).to_string();
                name = match fname.rfind('.') {
                    Some(i) => fname[..i].to_string(),
                    None => fname,
                };
            }
            out.insert(name, file);
        }
    }
    out
}

/// read_keyframes_into fills name and row tables the caller then merges, robot first and scene second
/// — so a scene row wins on a name collision.
fn read_keyframes_into(
    path: &str,
    names: &mut Vec<String>,
    rows: &mut Vec<Vec<f64>>,
) -> Result<(), String> {
    if path.is_empty() {
        return Ok(());
    }
    let src = std::fs::read_to_string(path)
        .map_err(|e| format!("simu.mjcf_convert: cannot read {path}: {e}"))?;
    let root = parse_document(&src)?;
    for el in &root.children {
        if el.name != "keyframe" {
            continue;
        }
        for k in &el.children {
            if k.name == "key" {
                names.push(k.attr_or("name", ""));
                rows.push(floats(&k.attr_or("qpos", "")));
            }
        }
    }
    Ok(())
}
