# control-model — the model basis of the control stack

MIT-licensed (see [`LICENSE`](LICENSE)).

Eleven modules, no plant, no engine, no control law:

```text
readers      xml         XmlNode + parse_document, the reader for URDF and MJCF
             urdf        the fixed serial chain: ChainJoint/ChainLink/UrdfChain,
                         load_urdf_chain, rpy_to_r, fk / tip_pose / Jacobians,
                         and the Z1 instance's urdf_path() / home_q()
             mjcf_model  the converted MJCF product: MjcfModel (URDF text +
                         sidecar: joint extras, sites, keyframes)
             mjcf_convert the MJCF -> URDF bridge (MjcfConverter)

kinematics   body_tree   BodyTree, the floating-root tree
             pga_layer   the pga-crate conversion layer (screws, rotors, pose error)
             kinematics  the pose/motor bridge (Kinematics)
             pga_fk      forward kinematics as a PGA motor chain (PgaFk)

dynamics     pga_dynamics  the fixed-base PGA dynamics (PgaDynamicsModel)
             tree_dynamics the floating-base tree dynamics (TreeDynamicsModel)

data         models      the model-data paths (models/: z1, unitree_g1, wall),
                         resolved through this crate's own CARGO_MANIFEST_DIR
```

It depends on the three crates below it — [`pga`](../pga),
[`control-math`](../control-math) and [`control-base`](../control-base) — and on
`roxmltree` for the XML parse. It does
NOT depend on the C ABI shim, and has no `build.rs`: this crate builds and tests
with no engine present. The one thing it takes from `control-base` is the
pose -> motor conversion the Plant contract also states, so the convention lives
once, in the lower crate, and `pga_layer::rotor_from_quat` and
`Kinematics::pose_to_motor` read the pair there.

The cluster is closed under itself: every `crate::` reference inside it points at
another member (`urdf -> xml`, `body_tree -> urdf/mjcf_model/xml`,
`tree_dynamics -> body_tree/pga_dynamics/pga_layer`, ...), and its only external
edges are the three crates below it and `roxmltree`. That is why it can be built,
tested and released alone.

`models/` — the URDF/MJCF sources and the meshes they reference — is here with it:
`urdf_path()` and `home_q()` resolve through `models/z1/`, and the rest of the
data through `models::*` (`g1_dir`, `g1_robot`, `g1_scene`, `g1_urdf`, `g1_meta`,
`wall`). Every path is built from this crate's own `CARGO_MANIFEST_DIR`, so the
paths hold wherever the crate is built from.

## Tests

Seven suites, all here. `tests/model.rs` is the pure slice — the XML reader's
accept/reject and `rpy_to_r`'s identity anchor — with no model file on disk.
(`serde_json` is a dependency: the sidecar gate decodes the emitted JSON and
compares VALUES rather than text.) The six that need a robot:
`urdf.rs`, `pga_layer.rs`, `pga_dynamics.rs`, `tree_dynamics.rs`, `mjcf.rs`,
`body_tree.rs`. They resolve `models/` through this crate's own
`CARGO_MANIFEST_DIR`, so they run with no engine present.

`make test` is the suite and then the comment rules over the tree, so the lint is
not a step anyone has to remember. The rules are `../comment-why`, a sibling
project that reads text and asks the compiler for nothing — which is what lets
the same rules also be an ordinary test here, `tests/comment_why.rs`, with no
nightly and no plugin. They decide three shapes: process narration and filler, a
comment line whose content words are all in the code below it, and a short doc
comment that re-says the item's own name. The rest is a reader's call, and
`make comments` prints that crate's local approximation — long, marker-free and
mostly the code's own words — as advice it never fails on. The rules are a
DEV-dependency: the model basis above is still the whole of what this crate
links.
