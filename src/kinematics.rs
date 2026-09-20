// kinematics.rs — the PGA pose error as a quantity: B_e = -2 log(M_d ~M). That is NOT the
// endpoint-referenced world error [dp; dtheta]; the two are different notions, and tests/pga_layer.rs
// pins the difference so neither is swapped in for the other.
//
// Why a pose becomes a motor HERE and not in this module: `control_base::plant` states it once
// (`motor_of_pose`, and `motor_of_rotor` for a caller already holding the algebra's own rotation), and a
// third name for the same product is a third spelling of the same convention.

use crate::pga_layer::pga_pose_error;
use pga::Multivector;

pub struct Kinematics;

impl Kinematics {
    pub fn motor_log_error(&self, target: Multivector, current: Multivector) -> Multivector {
        pga_pose_error(target, current)
    }

    pub fn bivector_norm(&self, b: Multivector) -> f64 {
        let p = b.bivector_part();
        let mut s = 0.0f64;
        for v in p {
            s += v * v;
        }
        s.sqrt()
    }
}
