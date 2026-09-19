// pga_dynamics.rs — the PGA dynamics layer checked by the rigid-body identities
// ID(q,0,qdd) = M(q)qdd + g(q) and ID(q,qd,0) = C(q,qd)qd + g(q), payload included, plus
// mass-matrix symmetry and the screw-bracket helper.

use control_math::mat::Mat;
use control_math::vec3::Vec3;
use control_model::pga_dynamics::{pga_cross_biv_vec, pga_world_inertia, PgaDynamicsModel};
use control_model::pga_layer::{pga_biv_to_axial, pga_vec_to_biv, rotor_from_mat};
use control_model::urdf::{load_urdf_chain, urdf_path};

fn pga_dyn_test_model() -> PgaDynamicsModel {
    let chain = load_urdf_chain(&urdf_path(), "link00", "link06").expect("z1 chain");
    PgaDynamicsModel::new(chain)
}

/// The module's central geometric claim, as a machine check (GA_PID_AUDIT.md #12): sandwiching the
/// COM-frame angular momentum bivector by the link's rotor must read back as `R * (Ic * omega_b)`.
#[test]
fn the_world_inertia_map_is_the_pga_conjugation_of_the_com_tensor() {
    let vals = [
        [0.011900, 0.000410, -0.000220],
        [0.000410, 0.011300, 0.000170],
        [-0.000220, 0.000170, 0.001460],
    ];
    let mut ic = Mat::zeros(3, 3);
    for (i, row) in vals.iter().enumerate() {
        for (j, v) in row.iter().enumerate() {
            ic.set(i, j, *v);
        }
    }
    let r = Mat::from_axis_angle(Vec3::new(0.31, -0.52, 0.79).normalized(), 0.83);
    let iw = pga_world_inertia(&r, &ic);
    let rot = rotor_from_mat(&r);
    let mut worst = 0.0f64;
    for w_b in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.42, -0.31, 0.85),
    ] {
        let h_b = pga_vec_to_biv(ic.mul_vec3(w_b).to_array(), [0.0, 0.0, 0.0]);
        let (h_w, _) = pga_biv_to_axial(rot.apply(h_b));
        let want = r.mul_vec3(ic.mul_vec3(w_b));
        worst = worst
            .max((h_w[0] - want.x).abs())
            .max((h_w[1] - want.y).abs())
            .max((h_w[2] - want.z).abs());
    }
    eprintln!(
        "world inertia: rotor conjugation against the 3x3 congruence, worst delta {worst:.3e} \n\
         (the map itself: I_w[0][0] {:.6} against R Ic R^T's)",
        iw.at(0, 0)
    );
    assert!(
        worst < 1e-15,
        "the rotor conjugation of the COM tensor and the 3x3 congruence differ by {worst:.3e}: \
         the world inertia map is no longer the PGA operation the module's header claims"
    );
}

#[test]
fn pga_dyn_cross_biv() {
    // cross(biv, v) == axial Omega x v (screw-bracket helper)
    let b = pga_vec_to_biv([1.0, 2.0, -0.5], [0.0, 0.0, 0.0]);
    let v = Vec3::new(0.3, 0.7, -1.1);
    let ga = pga_cross_biv_vec(b, v);
    let eu = Vec3::new(1.0, 2.0, -0.5).cross(v);
    assert!((ga.x - eu.x).abs() < 1e-12);
    assert!((ga.y - eu.y).abs() < 1e-12);
    assert!((ga.z - eu.z).abs() < 1e-12);
}

#[test]
fn pga_dyn_mass_matrix_spd() {
    let mut mdl = pga_dyn_test_model();
    let m = mdl.mass_matrix(&[0.35, 1.22, -1.48, 0.61, 0.0, 0.30]);
    for i in 0..6 {
        for j in 0..6 {
            assert!(
                (m.at(i, j) - m.at(j, i)).abs() < 1e-12,
                "M not symmetric at ({i},{j})"
            );
        }
    }
    assert!(m.at(0, 0) > 0.0);
    assert!(m.at(5, 5) > 0.0);
}

#[test]
fn pga_dyn_inverse_dynamics_identities() {
    // ID(q, qd, 0) = bias + gravity and ID(q, 0, qdd) = M·qdd + gravity, to machine precision
    let mut mdl = pga_dyn_test_model();
    let q = [0.4, 1.0, -1.2, 0.5, -0.3, 0.8];
    let qd = [0.7, -0.5, 1.1, 0.3, -0.9, 0.4];
    let qdd = [1.2, -0.8, 0.5, -1.5, 0.9, -0.3];
    let zero = [0.0; 6];
    let id_v = mdl.inverse_dynamics(&q, &qd, &zero);
    let cb = mdl.bias_torques(&q, &qd);
    let cg = mdl.gravity_torques(&q);
    let id_a = mdl.inverse_dynamics(&q, &zero, &qdd);
    let m = mdl.mass_matrix(&q);
    for i in 0..6 {
        assert!(
            (id_v[i] - (cb[i] + cg[i])).abs() < 1e-9,
            "ID(q,qd,0) identity at joint {i}"
        );
        let mut mq = 0.0;
        for j in 0..6 {
            mq += m.at(i, j) * qdd[j];
        }
        assert!(
            (id_a[i] - (mq + cg[i])).abs() < 1e-9,
            "ID(q,0,qdd) identity at joint {i}"
        );
    }
}

#[test]
fn pga_dyn_payload_identities() {
    // the same identities under payload, plus: the payload must actually change the gravity torques
    let mut mdl = pga_dyn_test_model();
    mdl.payload_mass = 2.5;
    mdl.payload_com = Vec3::new(0.03, -0.02, 0.05);
    let q = [-0.7, 0.9, -0.4, 1.4, 0.2, 0.5];
    let qd = [0.3, 0.6, -0.8, 1.0, -0.4, 0.9];
    let qdd = [0.2, -1.1, 0.7, 0.4, -0.6, 1.3];
    let zero = [0.0; 6];
    let id_v = mdl.inverse_dynamics(&q, &qd, &zero);
    let cb = mdl.bias_torques(&q, &qd);
    let cg = mdl.gravity_torques(&q);
    let id_a = mdl.inverse_dynamics(&q, &zero, &qdd);
    let m = mdl.mass_matrix(&q);
    for i in 0..6 {
        assert!(
            (id_v[i] - (cb[i] + cg[i])).abs() < 1e-9,
            "payload ID(q,qd,0) at {i}"
        );
        let mut mq = 0.0;
        for j in 0..6 {
            mq += m.at(i, j) * qdd[j];
        }
        assert!(
            (id_a[i] - (mq + cg[i])).abs() < 1e-9,
            "payload ID(q,0,qdd) at {i}"
        );
    }
    let mut bare = pga_dyn_test_model();
    let bg = bare.gravity_torques(&q);
    let mut moved = 0.0f64;
    for i in 0..6 {
        moved = moved.max((cg[i] - bg[i]).abs());
    }
    assert!(moved > 1e-3, "payload did not change the gravity torques");
}

#[test]
fn a_model_parameter_change_invalidates_the_frame_cache() {
    // the mismatch knobs must take effect at an UNCHANGED q, frame cache included
    let mut mdl = pga_dyn_test_model();
    let q = [0.4, 1.0, -1.2, 0.5, -0.3, 0.8];
    let before = mdl.inverse_dynamics(&q, &[0.0; 6], &[1.0; 6]);
    mdl.scale_inertia(2.0);
    let after = mdl.inverse_dynamics(&q, &[0.0; 6], &[1.0; 6]);
    let mut moved = 0.0f64;
    for i in 0..6 {
        moved = moved.max((after[i] - before[i]).abs());
    }
    assert!(
        moved > 1e-6,
        "doubling every link inertia changed nothing at the same q"
    );
}
