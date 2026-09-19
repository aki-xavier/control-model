// pga_layer.rs — the conversion layer between this layer's types and the projective GA crate:
// screw embed/readout, quaternion/matrix -> rotor, and the coordinate-free pose-error screw.
// Holds no types (urdf.rs, kinematics.rs, pga_fk.rs and pga_dynamics.rs carry the users).
//
// Conventions (pga crate, basis e1 e2 e3 e0 with e0^2 = 0): bivector parts are
// (b12, b13, b23, b01, b02, b03), Euclidean lines then ideal lines = v ^ e0; axial readout is
// (b23, -b13, b12); rotor(axis) = cos(t/2) - sin(t/2) (n I3), apply = M X M~; motor =
// translator . rotor (translate after rotate); log returns the HALVED screw bivector, so
// B_e = -2 log(M_d M~) is full-angle. Checks: tests/pga_layer.rs, tests/urdf.rs.

use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use pga::Multivector;

// ---- screw helpers ----------------------------------------------------------

/// pga_vec_to_biv embeds the axial vector (omega) and the translation (v ^ e0)
/// into the PGA bivector: parts (z, -y, x | v.x, v.y, v.z).
pub fn pga_vec_to_biv(omega: [f64; 3], v: [f64; 3]) -> Multivector {
    pga::mv_bivector(omega[2], -omega[1], omega[0], v[0], v[1], v[2])
}

pub fn pga_biv_to_axial(b: Multivector) -> ([f64; 3], [f64; 3]) {
    let p = b.bivector_part();
    ([p[2], -p[1], p[0]], [p[3], p[4], p[5]])
}

/// euc_part extracts the scalar + Euclidean-line part (a motor's rotation part).
pub(crate) fn euc_part(m: Multivector) -> Multivector {
    let p = m.bivector_part();
    m.grade(0)
        .add(pga::mv_bivector(p[0], p[1], p[2], 0.0, 0.0, 0.0))
}

pub fn vec3_from_pga_vec(m: Multivector) -> Vec3 {
    let v = m.vector_part();
    Vec3::new(v[0], v[1], v[2])
}

/// rotor_from_quat: quaternion -> rotor = w - sin(t/2) (n I3); bivector parts (z, -y, x).
pub fn rotor_from_quat(q: Quat) -> Multivector {
    pga::mv_scalar(q.w).sub(pga::mv_bivector(q.z, -q.y, q.x, 0.0, 0.0, 0.0))
}

pub fn rotor_from_mat(r: &Mat) -> Multivector {
    rotor_from_quat(Quat::from_mat3(r))
}

pub fn pga_pose_error(target: Multivector, cur: Multivector) -> Multivector {
    target.gp(cur.reverse()).log().scale(-2.0)
}
