// kinematics.rs — the PGA pose/motor bridge. The geometric error (the bivector
// B_e = -2 log(M_d ~M)) and the endpoint-referenced world error [dp; dtheta] are
// different notions, and both live here.

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

    /// The exact geometric pose-error bivector B_e = -2 log(M_d ~M).
    pub fn motor_log_error(&self, target: Multivector, current: Multivector) -> Multivector {
        pga_pose_error(target, current)
    }

    /// |B_e| over the 6 bivector components.
    pub fn bivector_norm(&self, b: Multivector) -> f64 {
        let p = b.bivector_part();
        let mut s = 0.0f64;
        for v in p {
            s += v * v;
        }
        s.sqrt()
    }
}
