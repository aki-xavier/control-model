// control-model — the model basis of the control stack, as a project of its own.
//
// Twelve modules, and every one of them describes the MACHINE rather than commanding it: the
// readers (`xml`, `urdf`, `mjcf_model`, `mjcf_convert`), the two kinematic shapes (the fixed serial
// chain and the floating-base `body_tree`), and their projective-GA kinematics and dynamics
// (`pga_layer`, `kinematics`, `pga_fk`, `pga_dynamics`, `tree_dynamics`). `vfmt` is here because the
// URDF/MJCF this crate emits is a committed artifact whose bytes are a contract.
//
// It carries no plant, no engine and no control law: every consumer in the stack — the observers,
// the task loops, the legged stack, the benches — sits above it and reaches the machine through
// these types. The C ABI shim is NOT a dependency, so this crate builds and tests with no MuJoCo
// present (the property simu's own build.rs could not have while these modules lived in it).
//
// It was extracted from the simu crate's `src/` once the dependency graph made the order obvious:
// the cluster is closed under itself (every `crate::` reference inside it points at another member),
// its external dependencies are only the two siblings below it plus `roxmltree`, and nothing above
// it can be reached from below. `models/` — the URDF/MJCF sources and their meshes — came along, so
// `urdf_path()` and `home_q()` resolve through this crate's own directory and a consumer reaches the
// rest of the data through `models::*`. simu consumes this as a sibling path dependency
// (`{ path = "../control-model" }`) and names these modules EXPLICITLY (`control_model::urdf`, ...);
// it re-exports none of them, so a reader sees where a model type comes from.

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
pub mod vfmt;
pub mod xml;
