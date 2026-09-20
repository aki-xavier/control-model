// pga_layer.rs — the PGA layer's tests: the rotor/quaternion conventions and the pose-error metric
// against the analytic rotvec. Per-link Jacobians are checked in tests/urdf.rs.

use control_base::plant::motor_of_pose;
use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use control_model::kinematics::Kinematics;
use control_model::pga_fk::PgaFk;
use control_model::pga_layer::{mat_from_rotor, pga_biv_to_axial, pga_pose_error, rotor_from_quat};
use control_model::urdf::{load_urdf_chain, urdf_path};

fn pga_test_chain() -> control_model::urdf::UrdfChain {
    load_urdf_chain(&urdf_path(), "link00", "link06").expect("z1 chain")
}

#[test]
fn pga_pose_conventions() {
    let q = Quat::from_mat3(&Mat::from_axis_angle(
        Vec3::new(1.0, 0.0, 0.0),
        std::f64::consts::PI / 6.0,
    ));
    let rot = rotor_from_quat(q);
    let t = pga::mv_scalar(1.0).gp(rot).to_matrix();
    assert!((t[5] - (std::f64::consts::PI / 6.0).cos()).abs() < 1e-9); // row1 col1
    assert!((t[9] - (std::f64::consts::PI / 6.0).sin()).abs() < 1e-9); // row2 col1
}

#[test]
fn pga_fk_tip_offset() {
    // the tool reach is this crate's own copy of the 0.051 m tip offset, so it is pinned here.
    let chain = pga_test_chain();
    let q = [0.4, 1.0, -1.2, 0.5, -0.3, 0.8];
    let (o, r) = chain.fk(&q);
    let tp = chain.tip_position(&o, &r);
    let i = o.len() - 1;
    let expect = o[i].add(r[i].mul_vec3(Vec3::new(chain.tool_reach, 0.0, 0.0)));
    assert!((tp.sub(expect)).norm() < 1e-12);
    // and the tip's ROTATION is the terminal frame's turned by the tool's own tip rotation
    let got_r = mat_from_rotor(&chain.tip_motor(&o, &r));
    let want_r = r[i].mul(&chain.tip_r);
    let mut worst = 0.0f64;
    for a in 0..3 {
        for b in 0..3 {
            worst = worst.max((got_r.at(a, b) - want_r.at(a, b)).abs());
        }
    }
    assert!(
        worst < 1e-12,
        "the tip rotation is {worst:.3e} off the tool's own"
    );
}

#[test]
fn pga_pose_error_metric() {
    // B_e = -2 log(M_d M~) matches the quaternion rotvec in magnitude AND direction
    let a = pga::motor([0.3, -0.2, 0.8], 0.7, [0.1, 0.02, -0.05]);
    let b = pga::motor([-0.4, 0.5, 0.2], -0.3, [-0.02, 0.05, 0.01]);
    let be = pga_pose_error(a, b);
    let (ax, _) = pga_biv_to_axial(be);
    let qa = Quat::from_mat3(&Mat::from_axis_angle(
        Vec3::new(0.3, -0.2, 0.8).normalized(),
        0.7,
    ));
    let qb = Quat::from_mat3(&Mat::from_axis_angle(
        Vec3::new(-0.4, 0.5, 0.2).normalized(),
        -0.3,
    ));
    let rv = Quat::rotvec_between(qa, qb);
    assert!((ax[0] - rv.x).abs() < 1e-9);
    assert!((ax[1] - rv.y).abs() < 1e-9);
    assert!((ax[2] - rv.z).abs() < 1e-9);
    let na: f64 = ax.iter().map(|v| v.abs()).sum();
    assert!(na > 1e-9);
}

/// The geometric pose error is NOT the law's pose error in any frame, and this pins why so it cannot
/// be read as a bug to fix: B_e = -2 log(M_d ~M) in the law's six slots stopped the arm ~0.78 m from
/// its target. The ROTATION half is a frame question and nothing more; the
/// TRANSLATION half is a MOMENT about the reference point (M = T(p) R factors the motor at the world
/// origin), so no frame readout turns one into the other. The swap was deleted rather than fixed.
#[test]
fn the_geometric_error_is_not_the_laws_error_in_any_frame() {
    let id = Quat::IDENTITY;
    let z90 = Quat::from_mat3(&Mat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5));
    let x35 = Quat::from_mat3(&Mat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), -0.35));
    // three readouts of one pose pair: the motor's at the world origin, the current frame's, and that
    // one carried into the world by the current rotation
    fn read(pd: Vec3, qd: Quat, pc: Vec3, qc: Quat) -> ([f64; 3], [f64; 3], [f64; 3], [f64; 3]) {
        let (ax_a, tr_a) =
            pga_biv_to_axial(pga_pose_error(motor_of_pose(pd, qd), motor_of_pose(pc, qc)));
        let (ax_b, tr_b) = pga_biv_to_axial(pga_pose_error(
            motor_of_pose(pc, qc).reverse(),
            motor_of_pose(pd, qd).reverse(),
        ));
        let rc = qc.to_mat3();
        let ax_c = rc.mul_vec3(Vec3::new(ax_b[0], ax_b[1], ax_b[2])).to_array();
        let tr_c = rc.mul_vec3(Vec3::new(tr_b[0], tr_b[1], tr_b[2])).to_array();
        (ax_a, tr_a, ax_c, tr_c)
    }
    // (1) and (3): a pure translation of +0.1 m on x — no rotation half, and the readout is -0.1
    let (ax_a, tr_a, _, _) = read(Vec3::new(0.1, 0.0, 0.0), id, Vec3::ZERO, id);
    for (i, v) in ax_a.iter().enumerate() {
        assert!(
            v.abs() < 1e-15,
            "a pure translation has a rotation half [{i}]: {ax_a:?}"
        );
    }
    assert!(
        (tr_a[0] + 0.1).abs() < 1e-15,
        "a pure translation of +0.1 m no longer reads -0.1: {tr_a:?}"
    );
    assert!(tr_a[1].abs() < 1e-15 && tr_a[2].abs() < 1e-15, "{tr_a:?}");
    // (2a) same position, different orientation: the tip does not move, the direct readout is not
    // zero (a moment about the world origin), and the current-point readouts are exactly zero
    let (_, tr2_a, _, tr2_c) = read(Vec3::new(0.2, 0.0, 0.0), z90, Vec3::new(0.2, 0.0, 0.0), id);
    assert!(
        tr2_a[1].abs() > 0.05,
        "the direct readout of a ZERO tip displacement is {tr2_a:?}: has it become a displacement?"
    );
    assert!(
        tr2_c.iter().all(|v| v.abs() < 1e-15),
        "a rotation about the current point reads {tr2_c:?} about the current point"
    );
    // (2b) the same rotation error read from two current positions: the readout follows the position,
    // which a displacement of a point cannot do
    let pd = Vec3::new(0.3, -0.1, 0.2);
    let (_, _, _, t_a) = read(pd, z90, Vec3::new(0.15, 0.05, -0.02), x35);
    let (_, _, _, t_b) = read(pd, z90, Vec3::new(-0.4, 0.25, 0.05), x35);
    let moved = t_a
        .iter()
        .zip(t_b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0f64, f64::max);
    assert!(
        moved > 0.1,
        "the same rotation error from two positions moved the translation half by only {moved:.2e}"
    );
    // (2c) the frame fix: the conjugation makes the ROTATION half the law's rotvec_between exactly,
    // while the TRANSLATION half's residual (0.472 m) stays larger than the 0.290 m displacement it
    // is supposed to be
    let (_, _, ax_c, tr_c) = read(pd, z90, Vec3::new(0.15, 0.05, -0.02), x35);
    let want = Quat::rotvec_between(z90, x35);
    for (i, (a, w)) in ax_c.iter().zip([want.x, want.y, want.z]).enumerate() {
        assert!(
            (a - w).abs() < 1e-12,
            "the world readout's rotation half is not the law's rotvec [{i}]: {ax_c:?} against ({:.4}, {:.4}, {:.4})",
            want.x,
            want.y,
            want.z
        );
    }
    let dp = pd.sub(Vec3::new(0.15, 0.05, -0.02));
    let res = tr_c
        .iter()
        .zip([dp.x, dp.y, dp.z])
        .map(|(x, y)| (x - y).abs())
        .fold(0.0f64, f64::max);
    assert!(
        res > 0.1,
        "the world readout's translation residual is {res:.3e} against a {:.3e} m displacement",
        dp.norm()
    );
}

#[test]
fn motor_log_error_and_bivector_norm_agree_with_the_pose_error() {
    // the Kinematics bridge is a namespace over the same two operations, so it must agree with the
    // free functions the layer exposes
    let k = Kinematics;
    let tgt = motor_of_pose(
        Vec3::new(0.1, -0.2, 0.3),
        Quat {
            w: 0.6f64.cos() / 2.0,
            x: 0.0,
            y: 0.0,
            z: 0.6f64.sin() / 2.0,
        },
    );
    let cur = motor_of_pose(Vec3::new(0.0, 0.0, 0.0), Quat::IDENTITY);
    let via_ns = k.motor_log_error(tgt, cur);
    let via_fn = pga_pose_error(tgt, cur);
    assert!(via_ns.approx_eq(via_fn));
    assert!((k.bivector_norm(via_ns) - k.bivector_norm(via_fn)).abs() < 1e-15);
    assert!(k.bivector_norm(via_ns) > 1e-3);
}

/// The third kinematics agrees with the chain, which had no machine check until this one: PgaFk::motor
/// is not a spare copy of UrdfChain::fk but a second implementation of the same recursion, so a
/// divergence would show up nowhere else. The motor's chain ends at the TERMINAL link frame, not at a
/// joint.
#[test]
fn the_motor_chain_and_the_matrix_chain_are_the_same_kinematics() {
    let chain = pga_test_chain();
    let fk = PgaFk::new(&urdf_path(), "link00", "link06").expect("z1 pga fk");
    // the home pose, a general one, and one with a joint near its own limit
    let postures: [&[f64]; 3] = [
        &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        &[0.4, 1.0, -1.2, 0.5, -0.3, 0.8],
        &[-1.1, 0.35, 2.4, -0.9, 0.6, -1.7],
    ];
    let k = Kinematics;
    let mut worst_pose = 0.0f64;
    let mut worst_pos = 0.0f64;
    let mut worst_rot = 0.0f64;
    for q in postures {
        let (o, r) = chain.fk(q);
        let i = o.len() - 1;
        let got = fk.motor(q);
        let want = motor_of_pose(o[i], Quat::from_mat3(&r[i]));
        worst_pose = worst_pose.max(k.bivector_norm(pga_pose_error(want, got)));
        let m = got.to_matrix();
        let t = Vec3::new(m[3], m[7], m[11]);
        worst_pos = worst_pos.max(t.sub(o[i]).norm());
        for row in 0..3 {
            for col in 0..3 {
                worst_rot = worst_rot.max((m[4 * row + col] - r[i].at(row, col)).abs());
            }
        }
    }
    eprintln!(
        "motor chain against matrix chain, worst over 3 postures: pose error {worst_pose:.3e}, \
         position {worst_pos:.3e} m, rotation {worst_rot:.3e}"
    );
    // the same recursion read two ways, so this is float noise rather than a tolerance: the
    // arithmetic differs (geometric products against 3x3 multiplies), which is the point of checking
    assert!(
        worst_pose < 1e-12 && worst_pos < 1e-12 && worst_rot < 1e-12,
        "the motor chain and the matrix chain disagree: pose {worst_pose:.3e}, position \
         {worst_pos:.3e} m, rotation {worst_rot:.3e} — the two kinematics are no longer the same \
         machine"
    );
}

/// The BASE-POSE reading, pinned to the matrix it replaced: `mat_from_rotor` must return EXACTLY what
/// `Quat::to_mat3` returns for the same four numbers, because a base pose arrives as a rotor now while
/// every Jacobian and inertia map downstream is still 3 x 3. The angles below span a vanishing
/// perturbation and a large one on purpose: a sign error in this reading is a rotation by the INVERSE
/// angle, which a random sample of postures can hide in a tolerance and a small angle cannot.
#[test]
fn mat_from_rotor_is_exactly_the_matrix_the_quaternion_path_wrote() {
    let mut worst_q = 0.0f64;
    let mut worst_m = 0.0f64;
    for angle in [1e-9, 1e-7, 0.2, std::f64::consts::FRAC_PI_2, 2.9] {
        for axis in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.31, -0.52, 0.79).normalized(),
        ] {
            let r = Mat::from_axis_angle(axis, angle);
            let q = Quat::from_mat3(&r);
            let via_q = q.to_mat3();
            let via_rotor = mat_from_rotor(&rotor_from_quat(q));
            let via_axis = mat_from_rotor(&pga::rotor(axis.to_array(), angle));
            for i in 0..3 {
                for j in 0..3 {
                    worst_q = worst_q.max((via_rotor.at(i, j) - via_q.at(i, j)).abs());
                    worst_m = worst_m.max((via_axis.at(i, j) - r.at(i, j)).abs());
                }
            }
        }
    }
    eprintln!(
        "the rotor reading against Quat::to_mat3: {worst_q:.3e}; against Mat::from_axis_angle: \
         {worst_m:.3e}"
    );
    assert!(
        worst_q == 0.0,
        "mat_from_rotor and Quat::to_mat3 differ by {worst_q:.3e}: the base-pose reading is no \
         longer the matrix the quaternion path wrote"
    );
    assert!(
        worst_m < 1e-12,
        "mat_from_rotor and Mat::from_axis_angle differ by {worst_m:.3e}"
    );
}
