# GeometricModelingPlugin 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `bin/*/plugins/com.cloudsim.geomodeling/` |
| 职责 | 几何建模工作区：Ribbon、PlaneGCS 草图、实体特征、特征树与 Undo |

## 2. 边界

- 形状真源：Data `ParametricBrepModel` + `rebuild()`（GeometryAlgorithm）
- 网页同核不同壳：见 [网页端 API](../../features/网页端/全量对等/API_CONTRACT.md)
- Host ABI ≥ 1.48.0

## 3. 已交付能力

见 [README.md](README.md) 与专题 [几何建模/FEATURES.md](../../features/几何建模/FEATURES.md)。

## 4. 验证

Debug|x64 + Release|x64 编本工程；依赖链 PluginSDK → GeometryAlgorithm → Data → Host → 本插件。
