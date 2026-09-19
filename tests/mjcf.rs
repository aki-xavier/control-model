// mjcf.rs — the MJCF -> URDF bridge's self-consistency suite: the converted model parses back
// through the project's own URDF pipeline, carries the model's physics, and puts the feet where the
// upstream 'stand' keyframe says they are. No external ground truth; one gate compares the emitted
// URDF's text against the artifact committed in models/, and another compares the sidecar's DECODED
// values (its rendering is Rust's `Display`, not pinned text).

use control_model::mjcf_convert::MjcfConverter;
use control_model::mjcf_model::MjcfModel;
use control_model::urdf::{load_urdf_chain, UrdfChain};
use control_model::xml::parse_document;
use std::path::PathBuf;

pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

pub fn g1_dir() -> PathBuf {
    repo_root().join("models").join("unitree_g1")
}

pub fn convert_model() -> MjcfModel {
    let dir = g1_dir();
    let mut conv = MjcfConverter::new();
    conv.convert(
        dir.join("src").join("g1.xml").to_str().unwrap(),
        dir.join("src").join("scene.xml").to_str().unwrap(),
    )
    .expect("g1 conversion")
}

/// Writes the converted URDF where load_urdf_chain can read it. ONE PATH PER CALL: the tests of a
/// binary run in parallel threads and a shared path let one read the file while another wrote it.
fn convert_urdf_path(m: &MjcfModel) -> PathBuf {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static SEQ: AtomicUsize = AtomicUsize::new(0);
    let n = SEQ.fetch_add(1, Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!("g1_convert_test_{}_{n}.urdf", std::process::id()));
    std::fs::write(&p, &m.urdf).expect("write converted urdf");
    p
}

fn chains(m: &MjcfModel) -> (UrdfChain, UrdfChain) {
    let up = convert_urdf_path(m);
    let up = up.to_str().unwrap().to_string();
    let left = load_urdf_chain(&up, "pelvis", "left_ankle_roll_link").expect("left leg chain");
    let right = load_urdf_chain(&up, "pelvis", "right_ankle_roll_link").expect("right leg chain");
    (left, right)
}

/// json_same compares two decoded documents by VALUE: numbers by their f64 value — `serde_json` keeps
/// each literal's own representation, so a committed `88.0` and a written `88` are the same number —
/// arrays elementwise, and objects key by key.
fn json_same(a: &serde_json::Value, b: &serde_json::Value) -> bool {
    use serde_json::Value;
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => match (x.as_f64(), y.as_f64()) {
            (Some(x), Some(y)) => x == y,
            _ => x == y,
        },
        (Value::Array(x), Value::Array(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(a, b)| json_same(a, b))
        }
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len() && x.iter().all(|(k, v)| y.get(k).is_some_and(|w| json_same(v, w)))
        }
        _ => a == b,
    }
}

/// The sidecar's numbers, not its text: the writer renders floats through Rust's `Display`, so the
/// gate decodes both documents and compares VALUES. A change of layout — or the loss of the old
/// pinned form — is not a failure as long as every number reads back the same double.
#[test]
fn the_sidecar_matches_the_committed_meta() {
    let m = convert_model();
    let want_text =
        std::fs::read_to_string(g1_dir().join("unitree_g1_meta.json")).expect("committed meta");
    let got: serde_json::Value =
        serde_json::from_str(&m.meta_json()).expect("the emitted sidecar parses as JSON");
    let want: serde_json::Value =
        serde_json::from_str(&want_text).expect("the committed meta parses as JSON");
    assert!(
        json_same(&got, &want),
        "the sidecar's decoded values differ from the committed meta"
    );
}

/// Pins a known divergence: the committed unitree_g1.urdf came from an OLDER converter (one
/// `_unit_box.stl` sole per foot) while the current one emits four `_unit_sphere.stl` colliders per
/// foot. The engine loads the COMMITTED file, so regenerating it is a project decision.
#[test]
fn the_committed_urdf_is_out_of_date_with_the_converter() {
    let m = convert_model();
    let committed =
        std::fs::read_to_string(g1_dir().join("unitree_g1.urdf")).expect("committed urdf");
    assert!(
        committed.contains("_unit_box.stl"),
        "the committed URDF no longer carries the box soles: regenerate it and flip this test to a byte-equality check"
    );
    assert!(
        m.urdf.contains("_unit_sphere.stl"),
        "the converter stopped emitting the unit-sphere soles"
    );
    // the rest of the two documents is the same generation, so with the collision blocks taken out
    // they must agree line for line (a plain compare cannot: the emitted file has four colliders
    // where the committed one has one, so every later line is offset)
    let a = strip_collisions(&m.urdf);
    let b = strip_collisions(&committed);
    assert_eq!(
        a.len(),
        b.len(),
        "outside the collision blocks the two documents have different line counts"
    );
    for i in 0..a.len() {
        assert_eq!(
            a[i], b[i],
            "outside the collision blocks the documents differ at line {}\n  rust:      {}\n  committed: {}",
            i + 1,
            a[i],
            b[i]
        );
    }
}

/// strip_collisions drops every `<collision>...</collision>` block, so the two
/// generations can be compared where they are meant to agree.
fn strip_collisions(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut inside = false;
    for line in text.lines() {
        if line.trim_start().starts_with("<collision") {
            inside = true;
        }
        if !inside {
            out.push(line.to_string());
        }
        if inside && line.trim_start().starts_with("</collision>") {
            inside = false;
        }
    }
    out
}

#[test]
fn leg_chains_parse_back_with_joint_facts() {
    let m = convert_model();
    let (left, right) = chains(&m);
    assert!(left.n == 6 && right.n == 6);
    let expected = [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ];
    for (i, name) in expected.iter().enumerate() {
        assert_eq!(left.joint_names[i], *name);
    }
    // the hip pitch and knee ranges as g1.xml states them, converted with full precision
    assert!((left.limit_lo[0] - (-2.5307)).abs() < 1e-9);
    assert!((left.limit_hi[0] - 2.8798).abs() < 1e-9);
    assert!((left.limit_lo[3] - (-0.087267)).abs() < 1e-9);
    assert!((left.limit_hi[3] - 2.8798).abs() < 1e-9);
    // the MJCF carries no per-joint damping, so what the converter writes is not a source fact
    for d in &left.dampings {
        assert!(*d >= 0.0);
    }
}

#[test]
fn arm_joints_are_present_in_model_order() {
    let m = convert_model();
    // The arm is not an all-revolute serial chain in this model (hand bodies hang off it), so it
    // cannot go through load_urdf_chain; what the conversion owes is the joints themselves, in order.
    let want = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ];
    let names: Vec<String> = m.joints.iter().map(|j| j.name.clone()).collect();
    let mut found = false;
    for i in 0..names.len().saturating_sub(want.len()) + 1 {
        if names[i..i + want.len()]
            .iter()
            .zip(want.iter())
            .all(|(a, b)| a == b)
        {
            found = true;
        }
    }
    assert!(
        found,
        "the left arm joints are not a run of the model joints: {names:?}"
    );
}

#[test]
fn total_mass_survives_the_conversion() {
    let m = convert_model();
    // sum every link mass straight out of the emitted URDF: the G1 is a 30-60 kg machine, and every
    // link has to carry a positive mass
    let root = parse_document(&m.urdf).expect("parse emitted urdf");
    let mut total = 0.0;
    let mut links = 0;
    for el in &root.children {
        if el.name != "link" {
            continue;
        }
        links += 1;
        for sub in &el.children {
            if sub.name != "inertial" {
                continue;
            }
            for leaf in &sub.children {
                if leaf.name == "mass" {
                    let mass: f64 = el_attr(leaf, "value", "0");
                    assert!(mass > 0.0);
                    total += mass;
                }
            }
        }
    }
    assert_eq!(links, 30, "pelvis + 29 child bodies");
    assert!(total > 30.0 && total < 60.0, "total mass {total}");
}

#[test]
fn sidecar_tables_carry_effort_sites_keyframes() {
    let m = convert_model();
    assert_eq!(m.joints.len(), 29);
    // the actuator force ranges as g1.xml states them: 88 N.m at the hip pitch, 139 N.m at the knee
    let hp = m.joint_extra("left_hip_pitch_joint");
    assert!((hp.effort_hi - 88.0).abs() < 1e-12);
    let kn = m.joint_extra("left_knee_joint");
    assert!((kn.effort_hi - 139.0).abs() < 1e-12);
    assert!(hp.armature >= 0.0 && kn.armature >= 0.0);
    let lf = m.site("left_foot");
    assert_eq!(lf.body, "left_ankle_roll_link");
    assert!(!m.kf_names.is_empty());
    let stand_i = m.kf_index("stand").expect("the stand keyframe");
    // the MJCF keyframe carries the free joint's 7 plus one slot per joint
    assert_eq!(m.kf_qpos[stand_i].len(), 36);
}

#[test]
fn stand_keyframe_puts_both_feet_at_one_height() {
    let m = convert_model();
    let (left, right) = chains(&m);
    let stand = m.kf_qpos[m.kf_index("stand").unwrap()].clone();
    // qpos layout: freejoint 7, then the joints in the engine's canonical order, whose two legs come
    // first: left leg 6, right leg 6
    let ql = &stand[7..13];
    let qr = &stand[13..19];
    let (ol, rl) = left.fk(ql);
    let (or_, rr) = right.fk(qr);
    let (ltip_p, _) = left.tip_pose(&ol, &rl);
    let (rtip_p, _) = right.tip_pose(&or_, &rr);
    // the upstream 'stand' is a level stance: both chain tips (the ankle link frames) must agree in
    // height and hang one leg-length below the pelvis. The 1e-6 m gate is the vendored MJCF's own
    // precision: its 6-digit quaternions are preserved verbatim, which moves the feet by ~1e-8 m.
    assert!((ltip_p.z - rtip_p.z).abs() < 1e-6);
    assert!(ltip_p.z < -0.3);
}

#[test]
fn sole_collision_geoms_survive_as_collision() {
    let m = convert_model();
    // The feet are this machine's contact role: the gate is that each foot link carries a <collision>
    // and that it is the unit-sphere geometry rather than the visual mesh.
    let mut n = 0;
    let root = parse_document(&m.urdf).expect("parse emitted urdf");
    for el in &root.children {
        if el.name != "link" || !el.attr_or("name", "").ends_with("ankle_roll_link") {
            continue;
        }
        for sub in &el.children {
            if sub.name != "collision" {
                continue;
            }
            for leaf in &sub.children {
                if leaf.name != "geometry" {
                    continue;
                }
                for g in &leaf.children {
                    if g.name == "mesh" && g.attr_or("filename", "").contains("_unit_sphere") {
                        assert!(g.attr_or("scale", "").starts_with("0.005"));
                        n += 1;
                    }
                }
            }
        }
    }
    // four sphere colliders per foot
    assert_eq!(n, 8);
}

#[test]
fn free_root_is_the_rootless_pelvis() {
    let m = convert_model();
    // the converter drops the synthetic world link: the engine injects its own free joint above the
    // rootless root link (a synthetic link plus an explicit floating joint doubled the root into two
    // free joints, measured at attach as "Pose size [14] does not match expected size [20]")
    assert!(!m.urdf.contains("<link name=\"world\"/>"));
    assert!(!m.urdf.contains("type=\"floating\""));
    assert!(m.urdf.contains("<link name=\"pelvis\">"));
}

// ---- helpers ----------------------------------------------------------------

/// el_attr reads an attribute with the `attr_or(..).f64()` fallback.
fn el_attr(n: &control_model::xml::XmlNode, key: &str, default: &str) -> f64 {
    n.attr_or(key, default).parse().unwrap_or(0.0)
}
