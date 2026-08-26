/// @file DEVELOPER_GUIDE.md
/// CollisionAlgorithm — 网格碰撞检测

# CollisionAlgorithm

内置 AABB 宽相 + 三角-三角窄相（可选 coal，见 `bin/SDK/coal/README.md`）。

## 公开 API

- `collision::CollisionWorld`：`upsertMeshBody` / `setWorldPose` / `setExcludePair` / `setSecurityMarginMm` / `checkAll`
- 单位：mm；`Mat4` 为列主序 4×4

## 内置窄相的已知限制

- **布尔判定**：`ContactHit::normal` / `depthMm` 为占位零值——内置路径不算穿透法向/深度，消费方勿拿来做碰撞响应；`pointMm` 仅命中两三角质心均值（诊断用）
- **共面漏检**：Möller 实现无共面分支，两三角共面（平行贴合/面内穿插）时仅投影点重合才报碰；装配"贴面"场景高频，需要真实共面检测请启用 coal 后端（`CLOUDSIM_HAS_COAL`，`hasCoalBackend()` 运行时可查）
- **复杂度**：无空间索引，AABB 重叠后 O(nA×nB) 全三角对
- **线程安全**：`CollisionWorld` 非线程安全，全部入口需调用方外部串行化

## 构建

`CollisionAlgorithm.vcxproj` → `CollisionAlgorithm.dll`（x64）
