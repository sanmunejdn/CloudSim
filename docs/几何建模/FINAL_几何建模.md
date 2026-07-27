# FINAL — 几何建模 SDK 插件

## 交付摘要

一期落地独立插件 `com.cloudsim.geomodeling`：菜单独占工作区（仅 AI + 中央建模页嵌 3D）、选面约束草图（PlaneGCS）、Pad/Pocket 特征树与 Undo，OCC 拉伸核在 GeometryAlgorithm，经 Host 1.21.0 API 暴露。

## 复制来源

| 组件 | 来源 | 许可 |
|------|------|------|
| PlaneGCS | OneCAD vendored / FreeCAD | LGPL-2.1+ |
| Extrude 核 | FreeCAD FeatureExtrude 路径 | LGPL-2.1+ |
| Command/特征语义 | OneCAD commands/history/sketch | MIT |

## 关键路径

- 插件：`src/Plugins/GeometricModelingPlugin/`
- 算法：`src/Geometry/GeometryAlgorithm/inc|source/SketchExtrude.*`
- 文档：`docs/几何建模/`
