// kinematics.rs — the PGA pose/motor bridge: a pose to a motor, and the geometric pose error
// B_e = -2 log(M_d ~M). That is NOT the endpoint-referenced world error [dp; dtheta]; the two are
// different notions, and tests/pga_layer.rs pins the difference so neither is swapped in for the
// other.

use crate::pga_layer::{pga_pose_error, rotor_from_quat};
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use pga::Multivector;

pub struct Kinematics;

impl Kinematics {
    /// Motor M = T(p) . R (rotate first, then translate).
    pub fn pose_to_motor(&self, position: Vec3, quaternion: Quat) -> Multivector {
        pga::translator(position.to_array()).gp(rotor_from_quat(quaternion))
    }

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
