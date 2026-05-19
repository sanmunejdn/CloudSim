# GeometryEngine conventions

> 模块说明与调用边界见 [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md)。

- Canonical type: `engine::RigidTransform` (`Eigen::Isometry3d`, translation in **mm**).
- Rotation truth: `Eigen::Quaterniond`; do not compare Euler angles for equality or IK residual.
- Scene / URDF / OSG chain (row vectors): `parent.composeScene(child)` ≡ `osgParent * osgChild`.
- `rigidTransformFromOsg` / `osgMatrixFromRigidTransform` transpose the 3×3 block (row-vector OSG ↔ column Eigen).
- Tool: `toolOriginFromFlange`, `flangeFromToolOrigin` — sole tool transform API; use `composeColumn` (not `composeScene`) so flange URDF→Eigen and tool matrices share one product rule.
- `RobotRigidFrame.positionMm` for `T_flange_tool` is the tool origin in **flange link axes** (URDF `flangeLinkName`), not base/world axes. Offset (0,0,-200) moves along flange Z; at poses where flange Z ∥ base Z it looks like world Z.
- Euler degrees: display and legacy JSON only (`eulerDegForDisplay`, `fromTranslationEulerDeg`).
- Legacy `BackendMat4`: use `colMajorFromRigidTransform` / `rigidTransformFromColMajor` at module boundaries only.
