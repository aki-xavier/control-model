// models.rs — where this crate's model DATA lives: the URDF/MJCF sources and the meshes they
// reference, under `models/` beside this crate. Each path is built from this crate's own
// CARGO_MANIFEST_DIR, so a consumer reaches the data through these functions instead of
// re-deriving a relative path from its own manifest — the data moved here with the model layer,
// and this is the only tree that holds a `models/` now.

use std::path::PathBuf;

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// root is the `models/` directory beside this crate.
pub fn root() -> PathBuf {
    manifest_dir().join("models")
}

/// z1_urdf is the Unitree Z1 arm's URDF — the fixed serial chain the arm benches and tests parse
/// (`urdf_path()` in `urdf.rs` is the same file as a `String`).
pub fn z1_urdf() -> PathBuf {
    root().join("z1").join("z1.urdf")
}

/// g1_dir is the Unitree G1's model directory: the MJCF sources, the committed URDF and the sidecar.
pub fn g1_dir() -> PathBuf {
    root().join("unitree_g1")
}

/// g1_robot is the G1's MJCF robot file (the converter's input).
pub fn g1_robot() -> PathBuf {
    g1_dir().join("src").join("g1.xml")
}

/// g1_scene is the G1's MJCF scene file — the keyframes the converter merges over the robot's.
pub fn g1_scene() -> PathBuf {
    g1_dir().join("src").join("scene.xml")
}

/// g1_urdf is the committed URDF the converter emits for the G1 (the byte-for-byte artifact).
pub fn g1_urdf() -> PathBuf {
    g1_dir().join("unitree_g1.urdf")
}

/// g1_meta is the committed sidecar the converter emits for the G1.
pub fn g1_meta() -> PathBuf {
    g1_dir().join("unitree_g1_meta.json")
}

/// wall is the static wall mesh the contact probes push against.
pub fn wall() -> PathBuf {
    root().join("wall.stl")
}
