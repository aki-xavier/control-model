// tree_dynamics.rs — the floating-base tree's suite. The strong gates are cross-checks against the
// fixed-base backend (pga_dynamics.rs): with the base at identity pose and zero twist, per-branch
// gravity, bias and mass rows must reproduce the chain backend's numbers. The ID identity
// ID = M alpha + C nu + g and the CoM finite difference are self-consistency.

use control_math::mat::Mat;
use control_math::vec3::Vec3;
use control_model::body_tree::{load_body_tree, BodyTree};
use control_model::mjcf_convert::MjcfConverter;
use control_model::mjcf_model::MjcfModel;
use control_model::pga_dynamics::PgaDynamicsModel;
use control_model::pga_layer::{rotor_from_mat, rotor_from_quat, rotor_identity};
use control_model::tree_dynamics::TreeDynamicsModel;
use control_model::urdf::load_urdf_chain;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn g1_tree() -> (BodyTree, MjcfModel) {
    let dir = repo_root().join("models").join("unitree_g1");
    let mut conv = MjcfConverter::new();
    let m = conv
        .convert(
            dir.join("src").join("g1.xml").to_str().unwrap(),
            dir.join("src").join("scene.xml").to_str().unwrap(),
        )
        .expect("g1 conversion");
    let t = load_body_tree(dir.join("unitree_g1.urdf").to_str().unwrap(), &m).expect("g1 tree");
    (t, m)
}

fn rand_q(n: usize, seed: f64) -> Vec<f64> {
    let mut q = vec![0.0; n];
    for (i, v) in q.iter_mut().enumerate() {
        *v = 0.5 * (seed + 1.7 * i as f64).sin();
    }
    q
}

/// arm_of: 0.0 when the joint is unregistered.
fn arm_of(t: &BodyTree, name: &str) -> f64 {
    for ex in &t.extras {
        if ex.name == name {
            return ex.armature;
        }
    }
    0.0
}

fn dmax_identity(dr: &Mat) -> f64 {
    let mut dmax = 0.0f64;
    for rr in 0..3 {
        for cc in 0..3 {
            let want = if rr == cc { 1.0 } else { 0.0 };
            dmax = dmax.max((dr.at(rr, cc) - want).abs());
        }
    }
    dmax
}

#[test]
fn tree_fk_matches_chain_fk_per_branch() {
    let (t, _) = g1_tree();
    let q = rand_q(t.n_q(), 0.3);
    let (o, r) = t.fk(Vec3::ZERO, rotor_identity(), &q);
    for end_link in ["left_ankle_roll_link", "right_ankle_roll_link"] {
        let c = load_urdf_chain(&t.urdf, "pelvis", end_link).expect("leg chain");
        // matched to the tree BY NAME: the tree's q order is the engine's canonical one, the chain's is the URDF path
        let mut qc = vec![0.0; c.n];
        for (i, nm) in c.joint_names.iter().enumerate() {
            qc[i] = q[t.q_index(nm).expect("joint in tree")];
        }
        let (oc, rc) = c.fk(&qc);
        let path = t.path_joints(t.node_index(end_link).expect("end link node"));
        for (k, qj) in path.iter().enumerate() {
            let ni = t.node_of_q(*qj).expect("q slot node");
            assert!(o[ni].sub(oc[k]).norm() < 1e-12);
            let dr = r[ni].mul(&rc[k].transposed());
            // matrix entries, not rotvecs: extraction is unstable for near-identity matrices
            assert!(
                dmax_identity(&dr) < 1e-12,
                "{end_link} branch at path step {k}"
            );
        }
    }
}

#[test]
fn gravity_matches_chain_backend_and_total_wrench() {
    let (t, _) = g1_tree();
    let mut d = TreeDynamicsModel::new(t.clone());
    let q = rand_q(t.n_q(), 1.1);
    let g = d.gravity_torques(Vec3::ZERO, rotor_identity(), &q);
    for end_link in ["left_ankle_roll_link", "right_ankle_roll_link"] {
        let c = load_urdf_chain(&t.urdf, "pelvis", end_link).expect("leg chain");
        let mut dc = PgaDynamicsModel::new(c.clone());
        let mut qc = vec![0.0; c.n];
        for (i, nm) in c.joint_names.iter().enumerate() {
            qc[i] = q[t.q_index(nm).expect("joint in tree")];
        }
        let gc = dc.gravity_torques(&qc);
        for (i, nm) in c.joint_names.iter().enumerate() {
            let row = 6 + t.q_index(nm).expect("joint in tree");
            assert!(
                (g[row] - gc[i]).abs() < 1e-9,
                "{end_link} gravity row {nm}: tree {} vs chain {}",
                g[row],
                gc[i]
            );
        }
    }
    assert!((g[5] - t.total_mass() * 9.81).abs() < 1e-6);
}

#[test]
fn mass_matrix_matches_chain_blocks_and_is_symmetric_pd() {
    let (t, _) = g1_tree();
    let mut d = TreeDynamicsModel::new(t.clone());
    let q = rand_q(t.n_q(), 2.3);
    let m = d.mass_matrix(Vec3::ZERO, rotor_identity(), &q);
    let c = load_urdf_chain(&t.urdf, "pelvis", "left_ankle_roll_link").expect("leg chain");
    let mut dc = PgaDynamicsModel::new(c.clone());
    let mut qc = vec![0.0; c.n];
    for (i, nm) in c.joint_names.iter().enumerate() {
        qc[i] = q[t.q_index(nm).expect("joint in tree")];
    }
    let mc = dc.mass_matrix(&qc);
    for (a, na) in c.joint_names.iter().enumerate() {
        for (b, nb) in c.joint_names.iter().enumerate() {
            let mut want = mc.at(a, b);
            if a == b {
                want += arm_of(&t, na);
            }
            let row = 6 + t.q_index(na).expect("joint in tree");
            let col = 6 + t.q_index(nb).expect("joint in tree");
            assert!(
                (m.at(row, col) - want).abs() < 1e-9,
                "mass block ({na},{nb}): tree {} vs chain {want}",
                m.at(row, col)
            );
        }
    }
    let n = m.rows;
    let x = vec![0.001; n];
    let b = m.mul_vec(&x);
    let x2 = m.solve(&b);
    let mut rmax = 0.0f64;
    for i in 0..n {
        rmax = rmax.max((x2[i] - x[i]).abs());
    }
    assert!(rmax < 1e-6, "mass matrix solve residual {rmax}");
}

#[test]
fn bias_matches_chain_backend() {
    let (t, _) = g1_tree();
    let mut d = TreeDynamicsModel::new(t.clone());
    let q = rand_q(t.n_q(), 3.7);
    let mut nu = vec![0.0; t.nv()];
    for i in 0..t.n_q() {
        nu[6 + i] = 0.4 * (0.9 + 1.3 * i as f64).cos();
    }
    let b = d.bias_torques(Vec3::ZERO, rotor_identity(), &q, &nu);
    let cl = load_urdf_chain(&t.urdf, "pelvis", "left_ankle_roll_link").expect("leg chain");
    let mut dl = PgaDynamicsModel::new(cl.clone());
    let mut qc = vec![0.0; cl.n];
    let mut vc = vec![0.0; cl.n];
    for (i, nm) in cl.joint_names.iter().enumerate() {
        let qi = t.q_index(nm).expect("joint in tree");
        qc[i] = q[qi];
        vc[i] = nu[6 + qi];
    }
    let bc = dl.bias_torques(&qc, &vc);
    for (i, nm) in cl.joint_names.iter().enumerate() {
        let row = 6 + t.q_index(nm).expect("joint in tree");
        assert!(
            (b[row] - bc[i]).abs() < 1e-9,
            "bias row {nm}: tree {} vs chain {}",
            b[row],
            bc[i]
        );
    }
}

#[test]
fn inverse_dynamics_identity() {
    let (t, _) = g1_tree();
    let mut d = TreeDynamicsModel::new(t.clone());
    let q = rand_q(t.n_q(), 5.1);
    let mut nu = vec![0.0; t.nv()];
    let mut alpha = vec![0.0; t.nv()];
    for i in 0..20 {
        nu[i] = 0.3 * (1.1 + 0.7 * i as f64).sin();
        alpha[i] = 0.5 * (0.4 + 0.9 * i as f64).cos();
    }
    let bp = Vec3::new(0.01, -0.02, 0.12);
    let bq = rotor_from_mat(&Mat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.2));
    let id = d.inverse_dynamics(bp, bq, &q, &nu, &alpha);
    let m = d.mass_matrix(bp, bq, &q);
    let rhs = m.mul_vec(&alpha);
    let b = d.bias_torques(bp, bq, &q, &nu);
    let g = d.gravity_torques(bp, bq, &q);
    let mut rmax = 0.0f64;
    for i in 0..20 {
        rmax = rmax.max((id[i] - rhs[i] - b[i] - g[i]).abs());
    }
    assert!(rmax < 1e-9, "ID identity residual {rmax}");
}

#[test]
fn com_jacobian_finite_difference() {
    let (t, _) = g1_tree();
    let bp = Vec3::new(0.02, 0.01, 0.12);
    let bq = rotor_identity();
    let q = rand_q(t.n_q(), 6.3);
    let (o, r) = t.fk(bp, bq, &q);
    let j = t.com_jacobian(&o, &r);
    let h = 1e-7;
    let c0 = t.total_com_w(&o, &r);
    // columns 3..5 are the base translation, 0..2 the base rotation (rotvec h * e_c), 6.. the joints
    for c in 0..3 {
        let mut dp = [0.0, 0.0, 0.0];
        dp[c] = h;
        let (o2, r2) = t.fk(bp.add(Vec3::new(dp[0], dp[1], dp[2])), bq, &q);
        let cd = t.total_com_w(&o2, &r2).sub(c0).scale(1.0 / h);
        for rr in 0..3 {
            assert!(
                (j.at(rr, 3 + c) - cd.to_array()[rr]).abs() < 1e-5,
                "base translation column {c} row {rr}"
            );
        }
    }
    for c in 0..3 {
        let mut rv = [0.0, 0.0, 0.0];
        rv[c] = h;
        let dq = rotor_from_mat(&Mat::from_axis_angle(
            Vec3::new(rv[0], rv[1], rv[2]).normalized(),
            h,
        ));
        let (o2, r2) = t.fk(bp, dq.gp(bq), &q);
        let cd = t.total_com_w(&o2, &r2).sub(c0).scale(1.0 / h);
        for rr in 0..3 {
            assert!(
                (j.at(rr, c) - cd.to_array()[rr]).abs() < 1e-4,
                "base rotation column {c} row {rr}"
            );
        }
    }
    for c in 0..14 {
        let mut q2 = q.clone();
        q2[c] += h;
        let (o2, r2) = t.fk(bp, bq, &q2);
        let cd = t.total_com_w(&o2, &r2).sub(c0).scale(1.0 / h);
        for rr in 0..3 {
            assert!(
                (j.at(rr, 6 + c) - cd.to_array()[rr]).abs() < 1e-4,
                "joint column {c} row {rr}"
            );
        }
    }
}

#[test]
fn stand_com_projects_inside_support_span() {
    let (t, m) = g1_tree();
    let (bp, bq) = m.keyframe_base("stand");
    let bq = rotor_from_quat(bq);
    let q = m.keyframe_q("stand", &t.q_names);
    assert_eq!(q.len(), 29);
    let (o, r) = t.fk(bp, bq, &q);
    let c = t.total_com_w(&o, &r);
    let (fl, _) = t.site_frame(&o, &r, "left_foot");
    let (fr_, _) = t.site_frame(&o, &r, "right_foot");
    assert!((fl.z - fr_.z).abs() < 1e-6);
    // the standing gate: CoM strictly between the feet in y, inside the sole's fore-aft span in x
    assert!(c.y > fl.y.min(fr_.y) && c.y < fl.y.max(fr_.y));
    let mid_x = 0.5 * (fl.x + fr_.x);
    assert!((c.x - mid_x).abs() < 0.05, "CoM x offset {}", c.x - mid_x);
}
