// urdf.rs — the chain layer's tests: per-link Jacobians against finite differences of FK, the tip pose
// against the terminal frame it is defined from, and rpy_to_r against the quaternion path. The damping
// fixture is the Z1 URDF's own values, verbatim.

use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use control_model::pga_layer::mat_from_rotor;
use control_model::urdf::{home_q, load_urdf_chain, rpy_to_r, urdf_path};

fn z1() -> control_model::urdf::UrdfChain {
    load_urdf_chain(&urdf_path(), "link00", "link06").expect("z1 chain parses")
}

#[test]
fn urdf_chain_parses_z1() {
    let chain = z1();
    assert_eq!(chain.n, 6);
    assert_eq!(chain.joint_names.len(), 6);
    // expected damping values from models/z1/z1.urdf
    assert!((chain.dampings[0] - 1.0).abs() < 1e-12);
    assert!((chain.dampings[1] - 2.0).abs() < 1e-12);
    assert!(chain.limit_lo[0] < chain.limit_hi[0]);
}

#[test]
fn link_jacobian_central_difference_matches() {
    let chain = z1();
    let home = home_q();
    let eps = 1e-6;
    let (o0, r0) = chain.fk(&home);
    for i in 1..6 {
        let j = chain.link_jacobian(&o0, &r0, i);
        for k in 0..6 {
            let mut qp = home.clone();
            let mut qm = home.clone();
            qp[k] += eps;
            qm[k] -= eps;
            let (op, _) = chain.fk(&qp);
            let (om, _) = chain.fk(&qm);
            let fd = [
                (op[i].x - om[i].x) / (2.0 * eps),
                (op[i].y - om[i].y) / (2.0 * eps),
                (op[i].z - om[i].z) / (2.0 * eps),
            ];
            for (r, v) in fd.iter().enumerate() {
                assert!(
                    (j.at(r, k) - v).abs() < 1e-5,
                    "central difference, link {i} joint {k} row {r}"
                );
            }
        }
    }
}

#[test]
fn an_unknown_end_link_is_an_error_but_a_sub_chain_is_not() {
    assert!(load_urdf_chain(&urdf_path(), "link00", "no_such_link").is_err());
    // a base link mid-arm is a legitimate request: the walk stops there and the chain is the sub-chain below it
    let sub = load_urdf_chain(&urdf_path(), "link03", "link06").expect("sub-chain");
    assert_eq!(sub.n, 3);
    assert_eq!(sub.joint_names[0], "joint4");
    assert_eq!(sub.child_names[2], "link06");
}

#[test]
fn point_jacobian_matches_finite_differences_of_fk() {
    let chain = z1();
    let q = home_q();
    let (o, r) = chain.fk(&q);
    let tip = chain.tip_position(&o, &r);
    let j = chain.point_jacobian(&o, &r, tip);
    let h = 1e-7;
    for c in 0..chain.n {
        let mut qp = q.clone();
        qp[c] += h;
        let (o2, r2) = chain.fk(&qp);
        let tip2 = chain.tip_position(&o2, &r2);
        let num = tip2.sub(tip).scale(1.0 / h);
        for (row, nv) in [num.x, num.y, num.z].iter().enumerate() {
            let analytic = j.at(row, c);
            assert!(
                (analytic - nv).abs() < 1e-6,
                "point jacobian, joint {c} row {row}: analytic {analytic} vs finite difference {nv}"
            );
        }
    }
}

#[test]
fn link_jacobian_matches_finite_differences_of_each_link_origin() {
    let chain = z1();
    let q = home_q();
    let (o, r) = chain.fk(&q);
    let h = 1e-7;
    for i in 0..chain.n {
        let j = chain.link_jacobian(&o, &r, i);
        for c in 0..chain.n {
            let mut qp = q.clone();
            qp[c] += h;
            let (o2, _) = chain.fk(&qp);
            let num = o2[i].sub(o[i]).scale(1.0 / h);
            for (row, nv) in [num.x, num.y, num.z].iter().enumerate() {
                let analytic = j.at(row, c);
                assert!(
                    (analytic - nv).abs() < 1e-6,
                    "link {i} jacobian, joint {c} row {row}: {analytic} vs {nv}"
                );
            }
            if c > i {
                assert_eq!(
                    j.at(0, c),
                    0.0,
                    "joint {c} moves link {i}'s origin in the model"
                );
            }
        }
    }
}

#[test]
fn full_jacobian_stacks_the_point_and_axis_blocks() {
    let chain = z1();
    let q = home_q();
    let (o, r) = chain.fk(&q);
    let tip = chain.tip_position(&o, &r);
    let full = chain.full_jacobian(&o, &r, tip);
    let point = chain.point_jacobian(&o, &r, tip);
    for c in 0..chain.n {
        for row in 0..3 {
            assert!((full.at(row, c) - point.at(row, c)).abs() < 1e-15);
        }
        let z = chain.world_z(&r, c);
        assert!((full.at(3, c) - z.x).abs() < 1e-15);
        assert!((full.at(4, c) - z.y).abs() < 1e-15);
        assert!((full.at(5, c) - z.z).abs() < 1e-15);
    }
}

#[test]
fn the_tip_readings_are_the_terminal_frame_plus_the_tip_and_tool_offsets() {
    let chain = z1();
    let (o, r) = chain.fk(&home_q());
    let p = chain.tip_position(&o, &r);
    let q = chain.tip_motor(&o, &r);
    let i = o.len() - 1;
    let off = chain.tip_p.add(Vec3::new(chain.tool_reach, 0.0, 0.0));
    let want = o[i].add(r[i].mul_vec3(off));
    assert!((p.sub(want)).norm() < 1e-15, "tip position {p:?}");
    let want_r = r[i].mul(&chain.tip_r);
    let got_r = mat_from_rotor(&q);
    for a in 0..3 {
        for b in 0..3 {
            assert!((got_r.at(a, b) - want_r.at(a, b)).abs() < 1e-15);
        }
    }
}

#[test]
fn rpy_to_r_is_the_extrinsic_xyz_rotation() {
    let id = rpy_to_r(&Vec3::new(0.0, 0.0, 0.0));
    for a in 0..3 {
        for b in 0..3 {
            assert!((id.at(a, b) - Mat::eye(3).at(a, b)).abs() < 1e-15);
        }
    }
    let angle = 0.6;
    let rz = rpy_to_r(&Vec3::new(0.0, 0.0, angle));
    let half = angle / 2.0;
    let q = Quat {
        w: half.cos(),
        x: 0.0,
        y: 0.0,
        z: half.sin(),
    };
    let qr = q.to_mat3();
    for a in 0..3 {
        for b in 0..3 {
            assert!(
                (rz.at(a, b) - qr.at(a, b)).abs() < 1e-15,
                "rpy_to_r yaw vs quaternion at ({a},{b})"
            );
        }
    }
    let m = rpy_to_r(&Vec3::new(0.3, -0.2, 0.7));
    let orth = m.mul(&m.transposed());
    for a in 0..3 {
        for b in 0..3 {
            let want = if a == b { 1.0 } else { 0.0 };
            assert!(
                (orth.at(a, b) - want).abs() < 1e-14,
                "not orthonormal at ({a},{b})"
            );
        }
    }
    let det = m.at(0, 0) * (m.at(1, 1) * m.at(2, 2) - m.at(1, 2) * m.at(2, 1))
        - m.at(0, 1) * (m.at(1, 0) * m.at(2, 2) - m.at(1, 2) * m.at(2, 0))
        + m.at(0, 2) * (m.at(1, 0) * m.at(2, 1) - m.at(1, 1) * m.at(2, 0));
    assert!((det - 1.0).abs() < 1e-14, "determinant {det}");
}
