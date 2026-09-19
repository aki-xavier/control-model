# control-model — the model basis of the control stack

A project of its own: `simu` (its consumer) depends on it as a sibling path
dependency, so no machine description lives in `simu`'s tree and this crate can be
built, tested and released alone. MIT-licensed (see `LICENSE`).

Eleven modules, no plant, no engine, no control law:

```text
readers      xml         XmlNode + parse_document, the reader for URDF and MJCF
             urdf        the fixed serial chain: ChainJoint/ChainLink/UrdfChain,
                         load_urdf_chain, rpy_to_r, fk / tip_pose / Jacobians
             mjcf_model  the converted MJCF product: MjcfModel (URDF text +
                         sidecar: joint extras, sites, keyframes)
             mjcf_convert the MJCF -> URDF bridge (MjcfConverter)

kinematics   body_tree   BodyTree, the floating-root tree (the biped's shape)
             pga_layer   the pga-crate conversion layer (screws, rotors, pose error)
             kinematics  the pose/motor bridge (Kinematics)
             pga_fk      forward kinematics as a PGA motor chain (PgaFk)

dynamics     pga_dynamics  the fixed-base PGA dynamics (PgaDynamicsModel)
             tree_dynamics the floating-base tree dynamics (TreeDynamicsModel)

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

The two items that did NOT travel are INSTANCE data rather than model data:
`urdf_path()` (which reads `simu`'s own `models/z1/`) and `home_q()` (that arm's
task start). They live in `simu`'s `urdf` facade over this crate. Two items that
were `pub(crate)` became `pub` because a caller above the boundary reads them:
`urdf::f64_attr` (simu's `sim_recorder`) and `PgaDynamicsModel`'s cached frames
plus `frames()` (simu's `plant/c_engine`). The arithmetic is unchanged; the
history of each file stays readable in `simu` (`git log --follow -- src/urdf.rs`).

## Tests

`tests/model.rs` is this crate's own slice — the formatting contract, the XML
reader's accept/reject, and `rpy_to_r`'s identity anchor — all pure, with no
model file on disk. The per-module oracles that need a robot (the Z1 chain, the
G1 tree, the dynamics identities) still live in `simu`'s `tests/` and reach these
modules through `simu`'s re-exports; they are run by `simu`'s own `make test`.
