// mjcf_model.rs — MjcfModel, the converted product of one MJCF robot file: the URDF document carries what URDF can
// express (chains, inertias, meshes, limits), the sidecar tables what URDF has no vocabulary for. Produced by MjcfConverter.

use control_math::quat::Quat;
use control_math::vec3::Vec3;
use serde::Serialize;
use std::collections::BTreeMap;

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

/// The sidecar's wire shape: three tables, each row a struct so its FIELD ORDER is the declaration
/// order (serde keeps it and no ordered map is needed); the one dynamic-key table is a BTreeMap,
/// whose keys come out sorted — the gate compares decoded values key by key, so that order is free.
#[derive(Serialize)]
struct MetaOut<'a> {
    joints: Vec<JointRow<'a>>,
    sites: Vec<SiteRow<'a>>,
    keyframes: BTreeMap<&'a str, &'a Vec<f64>>,
}

#[derive(Serialize)]
struct JointRow<'a> {
    name: &'a str,
    damping: f64,
    friction: f64,
    armature: f64,
    effort_lo: f64,
    effort_hi: f64,
}

#[derive(Serialize)]
struct SiteRow<'a> {
    name: &'a str,
    body: &'a str,
    pos: [f64; 3],
    quat: [f64; 4],
}

impl MjcfModel {
    /// meta_json renders the sidecar (joints, sites, keyframes) as JSON for the committed meta
    /// artifact; the URDF text travels as its own file. `serde_json` owns the encoding now: the
    /// numbers are ryu's shortest round-trip form, and the artifact is gated on its DECODED values
    /// (tests/mjcf.rs), so no text form is pinned.
    pub fn meta_json(&self) -> String {
        let joints = self
            .joints
            .iter()
            .map(|j| JointRow {
                name: &j.name,
                damping: j.damping,
                friction: j.friction,
                armature: j.armature,
                effort_lo: j.effort_lo,
                effort_hi: j.effort_hi,
            })
            .collect();
        let sites = self
            .sites
            .iter()
            .map(|s| SiteRow {
                name: &s.name,
                body: &s.body,
                pos: [s.pos.x, s.pos.y, s.pos.z],
                quat: [s.quat.w, s.quat.x, s.quat.y, s.quat.z],
            })
            .collect();
        let keyframes: BTreeMap<&str, &Vec<f64>> = self
            .kf_names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.as_str(), &self.kf_qpos[i]))
            .collect();
        let out = MetaOut {
            joints,
            sites,
            keyframes,
        };
        let mut s = serde_json::to_string_pretty(&out).expect("the sidecar serializes");
        s.push('\n');
        s
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
