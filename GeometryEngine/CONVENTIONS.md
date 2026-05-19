# GeometryEngine conventions

- Canonical type: `engine::RigidTransform` (`Eigen::Isometry3d`, translation in **mm**).
- Rotation truth: `Eigen::Quaterniond`; do not compare Euler angles for equality or IK residual.
- Scene / URDF / OSG chain (row vectors): `parent.composeScene(child)` ≡ `osgParent * osgChild`.
- `rigidTransformFromOsg` / `osgMatrixFromRigidTransform` transpose the 3×3 block (row-vector OSG ↔ column Eigen).
- Tool: `toolOriginFromFlange`, `flangeFromToolOrigin` — sole tool transform API.
- Euler degrees: display and legacy JSON only (`eulerDegForDisplay`, `fromTranslationEulerDeg`).
- Legacy `BackendMat4`: use `colMajorFromRigidTransform` / `rigidTransformFromColMajor` at module boundaries only.
