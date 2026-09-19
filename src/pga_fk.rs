// pga_fk.rs — PgaFk, forward kinematics of a serial chain as a PGA motor chain, parsed from URDF: the
// estimator's own motion model, which never peeks at the plant's true state (the matrix counterpart is UrdfChain::fk, urdf.rs).

use crate::pga_layer::rotor_from_mat;
use crate::urdf::{load_urdf_chain, UrdfChain};
use pga::Multivector;

/// PgaFk builds the end-effector motor M(q) from fixed joint-origin translators/rotors and the per-joint angle rotors.
#[derive(Clone, Debug)]
pub struct PgaFk {
    pub model: UrdfChain,
    translators: Vec<Multivector>,
    rpy_rotors: Vec<Multivector>,
}

impl PgaFk {
    pub fn new(urdf_path: &str, base_link: &str, end_link: &str) -> Result<PgaFk, String> {
        let model = load_urdf_chain(urdf_path, base_link, end_link)?;
        let mut translators = Vec::with_capacity(model.n);
        let mut rpy_rotors = Vec::with_capacity(model.n);
        for k in 0..model.n {
            translators.push(pga::translator(model.p_j[k].to_array()));
            rpy_rotors.push(rotor_from_mat(&model.r_j[k]));
        }
        Ok(PgaFk {
            model,
            translators,
            rpy_rotors,
        })
    }

    /// motor builds the end-effector motor M(q) from the base (world) frame; q is ordered like the chain's joint_names.
    pub fn motor(&self, q: &[f64]) -> Multivector {
        let mut m = pga::motor_identity();
        for k in 0..self.model.n {
            m = m
                .gp(self.translators[k])
                .gp(self.rpy_rotors[k])
                .gp(pga::rotor(self.model.axis[k].to_array(), q[k]));
        }
        m
    }
}
