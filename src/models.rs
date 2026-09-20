// models.rs — where this crate's model DATA lives: the URDF/MJCF sources and the meshes they
// reference, under `models/` beside this crate. Each path is built from this crate's own
// CARGO_MANIFEST_DIR, so the paths hold wherever the crate is built from.

use std::path::PathBuf;

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// The one path the rest of this file hangs off: the URDF/MJCF sources and the meshes beside this
/// crate.
pub fn root() -> PathBuf {
    manifest_dir().join("models")
}

pub fn z1_urdf() -> PathBuf {
    root().join("z1").join("z1.urdf")
}

pub fn g1_dir() -> PathBuf {
    root().join("unitree_g1")
}

/// The robot MJCF the converter reads, named as the first of the two documents it takes: the scene
/// beside it carries the keyframes, and the merge happens inside the conversion.
pub fn g1_robot() -> PathBuf {
    g1_dir().join("src").join("g1.xml")
}

/// g1_scene carries the keyframes the converter merges over the robot's.
pub fn g1_scene() -> PathBuf {
    g1_dir().join("src").join("scene.xml")
}

/// g1_urdf is the committed artifact, so it is regenerated rather than rewritten (tests/mjcf.rs).
pub fn g1_urdf() -> PathBuf {
    g1_dir().join("unitree_g1.urdf")
}

pub fn g1_meta() -> PathBuf {
    g1_dir().join("unitree_g1_meta.json")
}

pub fn wall() -> PathBuf {
    root().join("wall.stl")
}
