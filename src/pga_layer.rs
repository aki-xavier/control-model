// pga_layer.rs — the conversion layer between this crate's types and the projective GA crate: screw
// embed/readout, quaternion/matrix -> rotor, and the coordinate-free pose-error screw. It holds no
// types of its own — urdf.rs, kinematics.rs, pga_fk.rs and pga_dynamics.rs are its users.
//
// The quat -> rotor reading is RE-EXPORTED, not stated here: the Plant contract hands its poses out in
// the same rotor (`control_base::plant::rotor_of_quat`), so the one copy lives in the crate below and
// this name is that function.
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
/// into the PGA bivector, whose parts are therefore (z, -y, x | v.x, v.y, v.z).
pub fn pga_vec_to_biv(omega: [f64; 3], v: [f64; 3]) -> Multivector {
    pga::mv_bivector(omega[2], -omega[1], omega[0], v[0], v[1], v[2])
}

pub fn pga_biv_to_axial(b: Multivector) -> ([f64; 3], [f64; 3]) {
    let p = b.bivector_part();
    ([p[2], -p[1], p[0]], [p[3], p[4], p[5]])
}

/// The scalar + Euclidean-line part, which is a motor's rotation part: the contract's own read, aliased here
/// because the screw algebra below asks for it under this name.
pub(crate) use control_base::plant::motor_rotation as euc_part;

pub fn vec3_from_pga_vec(m: Multivector) -> Vec3 {
    let v = m.vector_part();
    Vec3::new(v[0], v[1], v[2])
}

/// rotor_from_quat: rotor = w - sin(t/2) (n I3), so the bivector parts are (z, -y, x).
///
/// Why this is a re-export and not a second reading of the same four numbers: the Plant contract hands
/// its poses out in this rotor too (`control_base::plant::rotor_of_quat`), so the ONE conversion lives
/// in the crate below and both sides are the same code. A convention with two spellings is two
/// conventions the day one of them moves.
pub use control_base::plant::rotor_of_quat as rotor_from_quat;

/// The same one conversion read the other way: the four numbers a rotor carries, so a caller still
/// speaking in quaternions reads them from the crate below rather than re-deriving the signs.
pub use control_base::plant::quat_of_rotor as quat_from_rotor;

/// The algebra's own identity rotation, re-exported where the other readings of a versor live: without
/// it every caller of a rotor-valued pose writes `mv_scalar(1.0)` for itself, and
/// `Multivector::default` is the algebra's ZERO — a zero rotor is not a rotation.
pub use pga::rotor_identity;

pub fn rotor_from_mat(r: &Mat) -> Multivector {
    rotor_from_quat(Quat::from_mat3(r))
}

/// mat_from_rotor: `rotor_from_mat`'s inverse reading — the rotation a versor carries.
///
/// Why it goes through the four numbers rather than reading the matrix off the blades itself: the sign
/// convention of a rotor IS the one `rotor_of_quat` writes down, and a second copy of it is how the two
/// come apart. A motor is read the same way and correctly: its ideal part is the TRANSLATION and leaves
/// the rotation alone.
pub fn mat_from_rotor(r: &Multivector) -> Mat {
    quat_from_rotor(*r).to_mat3()
}

/// To keep a 3 x 3 allocation out of a per-call loop, like `Quat::to_mat3_into`.
pub fn mat_from_rotor_into(r: &Multivector, out: &mut Mat) {
    quat_from_rotor(*r).to_mat3_into(out);
}

pub fn pga_pose_error(target: Multivector, cur: Multivector) -> Multivector {
    target.gp(cur.reverse()).log().scale(-2.0)
}
