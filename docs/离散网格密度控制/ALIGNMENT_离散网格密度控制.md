# 离散网格密度控制 - 对齐文档

## 1. 原始需求

Geometry 插件离散目前仅有 Coarse / Medium / Fine 三档。希望增加可配置能力：按目标三角面数，或按目标三角边长（mm）控制离散密度。

## 2. 项目上下文

- 算法：`geoalgo::discretizeShapeToMesh` → OCC `BRepMesh_IncrementalMesh`（线性/角偏差）
- 预设：`applyQualityPreset` 覆盖 `linearDeflectionMm` / `angularDeflectionDeg`
- 插件：`GeometryDockWidget` 质量下拉 → `PluginMeshDiscretizeParams.quality`
- 各向同性边长：`vcgalgo::isotropicRemesh`（Data 层已用，GeometryAlgorithm 不依赖 Vcg）

## 3. 边界确认

### 包含

- 保留三档预设
- 互斥模式：目标边长 / 目标三角面数
- SDK / Host / Data / GeometryAlgorithm / GeometryPlugin 参数透传与 UI
- SelfTest 与两份 DEVELOPER_GUIDE 更新

### 不包含

- `RemeshSoup` / `PointCloudSurface` 整模式
- BREP 导入固定 Medium 离散改动
- Tube/Ribbon 强制套用边长/面数模式
- 预估面数的实时联动

## 4. 需求理解

OCC 无法直接指定面数/边长。面数：对相对 deflection 二分；边长：OCC 基网格 + Data 层 `isotropicRemesh`。

## 5. 疑问澄清（已决策）

| 问题 | 决策 |
|------|------|
| UI 形态 | 预设 / 边长 / 面数三选一 |
| 边长实现 | OCC 基网格 + VCG remesh（Data） |
| 面数精度 | ±15% 或取最接近结果 |
