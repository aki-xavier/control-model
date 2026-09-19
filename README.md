# control-model — the model basis of the control stack

A project of its own: `simu` (its consumer) depends on it as a sibling path
dependency, so no machine description lives in `simu`'s tree and this crate can be
built, tested and released alone. MIT-licensed (see `LICENSE`).

Twelve modules, no plant, no engine, no control law:

```text
readers      xml         XmlNode + parse_document, the reader for URDF and MJCF
             urdf        the fixed serial chain: ChainJoint/ChainLink/UrdfChain,
                         load_urdf_chain, rpy_to_r, fk / tip_pose / Jacobians,
                         and the Z1 instance's urdf_path() / home_q()
             mjcf_model  the converted MJCF product: MjcfModel (URDF text +
                         sidecar: joint extras, sites, keyframes)
             mjcf_convert the MJCF -> URDF bridge (MjcfConverter)

kinematics   body_tree   BodyTree, the floating-root tree (the biped's shape)
             pga_layer   the pga-crate conversion layer (screws, rotors, pose error)
             kinematics  the pose/motor bridge (Kinematics)
             pga_fk      forward kinematics as a PGA motor chain (PgaFk)

dynamics     pga_dynamics  the fixed-base PGA dynamics (PgaDynamicsModel)
             tree_dynamics the floating-base tree dynamics (TreeDynamicsModel)

data         models      the model-data paths (models/: z1, unitree_g1, wall),
                         resolved through this crate's own CARGO_MANIFEST_DIR

format       vfmt        the pinned number/string formatting the emitted URDF,
                         the recorder's wire format and the bench JSON share
```

It depends on the two siblings below it — [`pga`](../pga) and
[`control-math`](../control-math) — and on `roxmltree` for the XML parse. It does
NOT depend on the C ABI shim, and has no `build.rs`: unlike `simu` (whose build
links `libeng_shim.dylib` for every target, the mathematical tests included),
this crate builds and tests with no engine present.

## Provenance

Extracted from `simu`'s `src/` at commit `5c56860`, where these files lived
beside the code that consumes them. It left now because the dependency graph
made the order obvious:

- the cluster is **closed under itself** — every `crate::` reference inside it
  points at another member (`urdf -> xml`, `body_tree -> urdf/mjcf_model/xml`,
  `tree_dynamics -> body_tree/pga_dynamics/pga_layer`, ...), so nothing inside
  reaches out to the layers above;
- its only external edges are the two crates below it and `roxmltree`;
- and it is the highest fan-in cluster in the tree (`urdf` is named by 14
  modules, `body_tree` by 9, `pga_dynamics` by 7) — the signature of a base
  layer, and the property [`control-math`](../control-math) and [`pga`](../pga)
  left on before it.

`models/` came along too: `urdf_path()` and `home_q()` resolve through this
crate's own `models/z1/` now, and a consumer reaches the rest of the data through
`models::*` (`g1_dir`, `g1_robot`, `g1_scene`, `g1_urdf`, `g1_meta`, `wall`)
instead of re-deriving a path from its own manifest — simu no longer holds a
`models/` of its own. Two items that were `pub(crate)` became `pub` because a
caller above the boundary reads them: `urdf::f64_attr` (simu's `sim_recorder`)
and `PgaDynamicsModel`'s cached frames plus `frames()` (simu's `plant/c_engine`).
The arithmetic is unchanged; the history of each file stays readable in `simu`
(`git log --follow -- src/urdf.rs`).

simu names these modules EXPLICITLY (`control_model::urdf`, ...): it re-exports
none of them, so a reader can see where a model type comes from.

## Tests

Seven suites, all here. `tests/model.rs` is the pure slice — the formatting
contract, the XML reader's accept/reject, and `rpy_to_r`'s identity anchor — with
no model file on disk. The six that need a robot moved here with the data:
`urdf.rs`, `pga_layer.rs`, `pga_dynamics.rs`, `tree_dynamics.rs`, `mjcf.rs`,
`body_tree.rs`. They resolve `models/` through this crate's own
`CARGO_MANIFEST_DIR`, so they run with no engine present; `simu`'s `make test`
invokes them by manifest path.
