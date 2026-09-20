// pga_fk.rs — PgaFk, forward kinematics of a serial chain as a PGA motor chain, parsed from URDF.
// It is the same recursion as UrdfChain::fk read through motors, so tests/pga_layer.rs checks the
// two against each other; the motor's chain ends at the TERMINAL link frame, not at a joint.

use crate::pga_layer::rotor_from_mat;
use crate::urdf::{load_urdf_chain, UrdfChain};
use pga::Multivector;

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

    /// q is ordered like the chain's joint_names.
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
