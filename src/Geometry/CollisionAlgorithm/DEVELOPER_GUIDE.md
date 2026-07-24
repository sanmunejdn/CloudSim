/// @file DEVELOPER_GUIDE.md
/// CollisionAlgorithm — 网格碰撞检测

# CollisionAlgorithm

内置 AABB 宽相 + 三角-三角窄相（可选 coal，见 `bin/SDK/coal/README.md`）。

## 公开 API

- `collision::CollisionWorld`：`upsertMeshBody` / `setWorldPose` / `setExcludePair` / `setSecurityMarginMm` / `checkAll`
- 单位：mm；`Mat4` 为列主序 4×4

## 构建

`CollisionAlgorithm.vcxproj` → `CollisionAlgorithm.dll`（x64）
