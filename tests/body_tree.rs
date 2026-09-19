// body_tree.rs — the G1's tree suite: the vendored MJCF must convert into a URDF the project's
// own pipeline parses back, and the BodyTree must carry the whole 29-joint floating-base
// humanoid with the engine-canonical q order.

use control_math::quat::Quat;
use control_math::vec3::Vec3;
use control_model::body_tree::{load_body_tree, BodyTree};
use control_model::mjcf_convert::MjcfConverter;
use control_model::mjcf_model::MjcfModel;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn g1_dir() -> PathBuf {
    repo_root().join("models").join("unitree_g1")
}

fn g1_model() -> MjcfModel {
    let dir = g1_dir();
    let mut conv = MjcfConverter::new();
    conv.convert(
        dir.join("src").join("g1.xml").to_str().unwrap(),
        dir.join("src").join("scene.xml").to_str().unwrap(),
    )
    .expect("g1 conversion")
}

fn g1_tree() -> BodyTree {
    let m = g1_model();
    let urdf = g1_dir().join("unitree_g1.urdf");
    load_body_tree(urdf.to_str().unwrap(), &m).expect("g1 tree")
}

#[test]
fn g1_has_29_joints_on_a_rootless_pelvis() {
    let m = g1_model();
    assert_eq!(m.joints.len(), 29);
    assert!(!m.urdf.contains("<link name=\"world\"/>"));
    assert!(!m.urdf.contains("type=\"floating\""));
    assert!(m.urdf.contains("<link name=\"pelvis\">"));
    let t = g1_tree();
    assert_eq!(t.q_names.len(), 29);
    assert_eq!(t.nodes.len(), 30); // pelvis + 29 child bodies
    assert_eq!(t.nodes[t.root].name, "pelvis");
}

#[test]
fn g1_engine_order_q_names() {
    let t = g1_tree();
    let expected = [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ];
    assert_eq!(t.q_names, expected);
}

#[test]
fn g1_stands_on_both_feet_at_zero_q() {
    let t = g1_tree();
    let (o, r) = t.fk(Vec3::ZERO, Quat::IDENTITY, &vec![0.0; 29]);
    // the all-zero posture of a humanoid MJCF is the straight stand: feet ~0.76 m below the pelvis
    // (measured), level to conversion precision, foot sites at the ankle origins
    let l_i = t.node_index("left_ankle_roll_link").expect("left ankle");
    let r_i = t.node_index("right_ankle_roll_link").expect("right ankle");
    assert!(l_i > 0 && r_i > 0);
    assert!(o[l_i].z < -0.5);
    assert!(o[r_i].z < -0.5);
    assert!((o[l_i].z - o[r_i].z).abs() < 1e-6);
    let (fl, _) = t.site_frame(&o, &r, "left_foot");
    let (fr_, _) = t.site_frame(&o, &r, "right_foot");
    assert!((fl.z - o[l_i].z).abs() < 1e-9);
    let c = t.total_com_w(&o, &r);
    assert!(c.z > o[l_i].z);
    assert!(c.z > -0.2);
    assert!(c.y > fl.y.min(fr_.y) && c.y < fl.y.max(fr_.y));
}

#[test]
fn the_tree_mass_is_the_sum_of_its_links() {
    let t = g1_tree();
    let total = t.total_mass();
    // the G1 is a 35 kg class humanoid; the MJCF inertials sum to 47.4 kg (the model's own tags)
    assert!(total > 30.0 && total < 60.0, "tree mass {total}");
    let mut by_hand = 0.0;
    for nd in &t.nodes {
        by_hand += nd.mass;
    }
    assert!((total - by_hand).abs() < 1e-12);
}

#[test]
fn press_sign_follows_the_joints_of_this_model() {
    // toe and heel opposite, the two legs mirrored
    let t = g1_tree();
    let (_, r) = t.fk(Vec3::ZERO, Quat::IDENTITY, &vec![0.0; 29]);
    let la = t
        .q_index("left_ankle_pitch_joint")
        .expect("left ankle pitch");
    let ra = t
        .q_index("right_ankle_pitch_joint")
        .expect("right ankle pitch");
    let toe = control_math::vec3::Vec3::new(1.0, 0.0, 0.0);
    let heel = control_math::vec3::Vec3::new(-1.0, 0.0, 0.0);
    assert_eq!(t.press_sign(&r, la, toe), -t.press_sign(&r, la, heel));
    assert_eq!(t.press_sign(&r, ra, toe), -t.press_sign(&r, ra, heel));
    assert_eq!(t.press_sign(&r, la, toe), t.press_sign(&r, ra, toe));
}

#[test]
fn the_com_jacobian_matches_finite_differences_of_the_com() {
    // the mixed base/joint Jacobian is what the balance law projects through: a sign or ordering
    // error in its base block is invisible in a pose check
    let t = g1_tree();
    let q = vec![0.0; 29];
    let base_p = Vec3::ZERO;
    let base_q = Quat::IDENTITY;
    let (o, r) = t.fk(base_p, base_q, &q);
    let j = t.com_jacobian(&o, &r);
    let c0 = t.total_com_w(&o, &r);
    let h = 1e-6;
    for k in 0..29 {
        let mut qp = q.clone();
        qp[k] += h;
        let (op, rp) = t.fk(base_p, base_q, &qp);
        let c1 = t.total_com_w(&op, &rp);
        let fd = c1.sub(c0).scale(1.0 / h);
        for (row, v) in [fd.x, fd.y, fd.z].iter().enumerate() {
            assert!(
                (j.at(row, 6 + k) - v).abs() < 1e-5,
                "com jacobian joint {k} row {row}: {} vs {v}",
                j.at(row, 6 + k)
            );
        }
    }
    // base linear columns are the identity: dc = dv for a pure translation
    for rr in 0..3 {
        for cc in 0..3 {
            let want = if rr == cc { 1.0 } else { 0.0 };
            assert!((j.at(rr, 3 + cc) - want).abs() < 1e-15);
        }
    }
}

/// THE CONTACT TABLE'S SLOT LAYOUT IS THE ABI'S, AND A CALLER MUST ASK BY NAME.
///
/// `BodyTree::contact_slot_of` turns a link name into the slot the engine's per-link contact table
/// reports it under (slot = node - 1, with the floating root — which has no chain slot — on the
/// last one). The layout is pinned against THIS model: the two feet — `swing_l`/`swing_r`, the
/// links that swing AND carry the whole weight — ARE slots 5 and 11, and a link added anywhere
/// before them would move the readout to another pair silently.
#[test]
fn the_contact_slot_layout_is_what_the_named_feet_resolve_to() {
    let t = g1_tree();
    let l = t.contact_slot_of("left_ankle_roll_link");
    let r = t.contact_slot_of("right_ankle_roll_link");
    assert_eq!(l, Some(5), "the left foot's contact slot moved");
    assert_eq!(r, Some(11), "the right foot's contact slot moved");
    // two feet, not the same reading twice; an unknown link answers None rather than another
    // link's row; and the floating root, which has no chain slot, rides the last one
    assert_ne!(l, r);
    assert_eq!(t.contact_slot_of("no_such_link"), None);
    assert_eq!(t.contact_slot_of("pelvis"), Some(t.nodes.len() - 1));
}
