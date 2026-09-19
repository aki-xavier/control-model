// mjcf_model.rs — MjcfModel, the converted product of one MJCF robot file: the URDF document carries what URDF can
// express (chains, inertias, meshes, limits), the sidecar tables what URDF has no vocabulary for. Produced by MjcfConverter.

use control_math::quat::Quat;
use control_math::vec3::Vec3;

/// MjcfSite is a named MJCF <site>: a task point welded to a body (the G1's left_foot/right_foot soles, an imu frame, a mouth tip); sites are not URDF concepts, so they ride the sidecar.
#[derive(Clone, Debug, Default)]
pub struct MjcfSite {
    pub name: String,
    pub body: String,
    pub pos: Vec3,
    pub quat: Quat,
}

/// MjcfJointExtra carries the per-joint dynamics URDF cannot express: armature (reflected rotor inertia — 0.0018
/// kg.m^2 against link inertias of ~1e-6, so it must reach the dynamics model) and effort (the actuator's forcerange).
#[derive(Clone, Debug, Default)]
pub struct MjcfJointExtra {
    pub name: String,
    pub damping: f64,
    pub friction: f64,
    pub armature: f64,
    pub effort_lo: f64,
    pub effort_hi: f64,
}

/// MjcfModel is the converter's output bundle: the URDF text plus the sidecar (joint extras, sites, keyframes — the scene's upstream-tuned STAND posture, the biped suite's simulation truth).
#[derive(Clone, Debug, Default)]
pub struct MjcfModel {
    pub urdf: String,
    pub joints: Vec<MjcfJointExtra>,
    pub sites: Vec<MjcfSite>,
    pub kf_names: Vec<String>,
    pub kf_qpos: Vec<Vec<f64>>,
}

/// vnum renders an f64 through Rust's own `Display` (the shortest round-trip form). The sidecar is a
/// committed artifact, but it is gated on its DECODED values rather than its bytes (tests/mjcf.rs),
/// so no form is pinned here: whatever shortest form `Display` spells, it reads back the same double.
fn vnum(x: f64) -> String {
    x.to_string()
}

impl MjcfModel {
    /// meta_json renders the sidecar (joints, sites, keyframes) as JSON for the committed meta artifact; the URDF text travels as its own file.
    pub fn meta_json(&self) -> String {
        let mut sb = String::with_capacity(1024);
        sb.push_str("{\n\"joints\": [\n");
        for (i, j) in self.joints.iter().enumerate() {
            let sep = if i + 1 < self.joints.len() { "," } else { "" };
            sb.push_str(&format!(
                "\t{{\"name\": \"{}\", \"damping\": {}, \"friction\": {}, \"armature\": {}, \"effort_lo\": {}, \"effort_hi\": {}}}{}\n",
                j.name,
                vnum(j.damping),
                vnum(j.friction),
                vnum(j.armature),
                vnum(j.effort_lo),
                vnum(j.effort_hi),
                sep
            ));
        }
        sb.push_str("],\n\"sites\": [\n");
        for (i, s) in self.sites.iter().enumerate() {
            let sep = if i + 1 < self.sites.len() { "," } else { "" };
            sb.push_str(&format!(
                "\t{{\"name\": \"{}\", \"body\": \"{}\", \"pos\": [{}, {}, {}], \"quat\": [{}, {}, {}, {}]}}{}\n",
                s.name,
                s.body,
                vnum(s.pos.x),
                vnum(s.pos.y),
                vnum(s.pos.z),
                vnum(s.quat.w),
                vnum(s.quat.x),
                vnum(s.quat.y),
                vnum(s.quat.z),
                sep
            ));
        }
        sb.push_str("],\n\"keyframes\": {\n");
        for (idx, name) in self.kf_names.iter().enumerate() {
            let q = &self.kf_qpos[idx];
            let sep = if idx + 1 < self.kf_names.len() {
                ","
            } else {
                ""
            };
            sb.push_str(&format!("\t\"{name}\": ["));
            for (i, v) in q.iter().enumerate() {
                let sep2 = if i + 1 < q.len() { ", " } else { "" };
                sb.push_str(&format!("{}{sep2}", vnum(*v)));
            }
            sb.push_str(&format!("]{sep}\n"));
        }
        sb.push_str("}\n}\n");
        sb
    }

    /// kf_index finds a keyframe by name (None for an absent name; the callers all guard).
    pub fn kf_index(&self, name: &str) -> Option<usize> {
        self.kf_names.iter().position(|n| n == name)
    }

    /// keyframe_q maps a keyframe's qpos (MJCF layout: freejoint 7, then joints in MJCF document order) into an
    /// arbitrary q order by joint name; empty when the keyframe or the joint table is missing.
    pub fn keyframe_q(&self, name: &str, q_names: &[String]) -> Vec<f64> {
        let Some(kfi) = self.kf_index(name) else {
            return Vec::new();
        };
        let kf = &self.kf_qpos[kfi];
        if kf.len() != 7 + self.joints.len() {
            return Vec::new();
        }
        let mut out = vec![0.0; q_names.len()];
        for (i, jn) in q_names.iter().enumerate() {
            for (k, mj) in self.joints.iter().enumerate() {
                if mj.name == *jn {
                    out[i] = kf[7 + k];
                }
            }
        }
        out
    }

    /// keyframe_base splits a keyframe's base pose (position + wxyz quaternion).
    pub fn keyframe_base(&self, name: &str) -> (Vec3, Quat) {
        let Some(kfi) = self.kf_index(name) else {
            return (Vec3::ZERO, Quat::IDENTITY);
        };
        let kf = &self.kf_qpos[kfi];
        if kf.len() < 7 {
            return (Vec3::ZERO, Quat::IDENTITY);
        }
        (
            Vec3::new(kf[0], kf[1], kf[2]),
            Quat {
                w: kf[3],
                x: kf[4],
                y: kf[5],
                z: kf[6],
            },
        )
    }

    /// joint_extra looks up one joint's sidecar row (zero values when absent).
    pub fn joint_extra(&self, name: &str) -> MjcfJointExtra {
        for j in &self.joints {
            if j.name == name {
                return j.clone();
            }
        }
        MjcfJointExtra {
            name: name.to_string(),
            ..Default::default()
        }
    }

    /// site looks up a named site (zero value when absent).
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
}
