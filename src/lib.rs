// control-model — the model basis of the control stack, as a project of its own.
//
// Eleven modules, and every one of them describes the MACHINE rather than commanding it: the
// readers (`xml`, `urdf`, `mjcf_model`, `mjcf_convert`), the two kinematic shapes (the fixed serial
// chain and the floating-base `body_tree`), and their projective-GA kinematics and dynamics
// (`pga_layer`, `kinematics`, `pga_fk`, `pga_dynamics`, `tree_dynamics`).
//
// It carries no plant, no engine and no control law, and the C ABI shim is NOT a dependency: that
// is why this crate builds and tests with no MuJoCo present, while the modules that do reach the
// engine pay for it in every target. The cluster is closed under itself — every `crate::` reference
// inside it points at another member, and its external dependencies are only the two crates below
// it plus `roxmltree` — so it can be built, tested and released alone. `models/` — the URDF/MJCF
// sources and their meshes — came along, so `urdf_path()` and `home_q()` resolve through this
// crate's own directory and the rest of the data through `models::*`.

pub mod body_tree;
pub mod kinematics;
pub mod mjcf_convert;
pub mod mjcf_model;
pub mod models;
pub mod pga_dynamics;
pub mod pga_fk;
pub mod pga_layer;
pub mod tree_dynamics;
pub mod urdf;
pub mod xml;
