# unitree_g1 模型(上游 MuJoCo Menagerie,BSD-3)

来源:[google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
的 `unitree_g1/`(Unitree Robotics 官方授权的 MJCF 描述)。对应宇树 G1
人形平台(G1 Plus 的 29 自由度核心构型):1.34 m、~35 kg、29 关节
(每腿 6 + 腰 3 + 每臂 7),浮动基座,足底为 4 枚 5 mm 接触球/足。

- `src/`:上游 MJCF 原文(g1.xml、g1_with_hands.xml、scene*.xml、
  README/CHANGELOG、LICENSE)。
- `meshes/`:51 个 STL(视觉 + 碰撞)及 `_unit_sphere.stl`(封闭单位球,
  足底球几何的载体——引擎 URDF 导入器只吃网格,不吃球体原语)。
- `unitree_g1.urdf`、`unitree_g1_meta.json`:**生成产物**,由 `src/mjcf_convert.rs`
  的 MjcfConverter 从 MJCF 转换;重新生成:
  `mbx run --release --example export_unitree_g1_urdf`。

接入验证:`tests/mjcf.rs`(转换/质量/限位/站姿足位)、`tests/body_tree.rs`(引擎 q 序)、
`tests/g1_attach.rs`(引擎 35 DOF 浮动基座、FK 一致、足底球触地注册)。播放器:`?rec=g1`。
