# FINAL — 三维模型 → 二维工程图

## 交付摘要

独立插件 `EngineeringDrawingPlugin`（`com.cloudsim.drawing`）已完成一期 MVP 与二期增强：B-rep → OCC HLR 工程图（三视图 / 可选轴测与剖视）、图幅交互（缩放平移、拖视图、线性尺寸、局部放大）、SVG/DXF 导出，以及宿主通用侧栏 API。

## 一期要点

- 算法：`geoalgo::projectShapeHlr` / 三视图（`HlrProject.*`，`TKHLR`）
- UI：菜单「工程图」、`DrawingSheetCanvasWidget`、侧车 `engineeringDrawing`
- 布局：归一化后第一角法网格，可见实线 / 隐藏虚线

## 二期要点

| 能力 | 实现 |
|------|------|
| 通用侧栏 | `enterAlternateSideUi` / `exitAlternateSideUi`（1.34.0）；旧 `enterProcessFlowSideUi` 转发 |
| 投影 | `PluginDrawingProjectParams` + `projectBrepToEngineeringDrawing`；第一/第三角 |
| 轴测 / 剖视 | `HlrViewKind::Iso`；中面 `BRepAlgoAPI_Section` → HLR |
| 标注 / 拖动 / 局部放大 | 画布工具：选择拖视图、线性尺寸两点、拖框细节视图 ×2 |
| 导出 | `DrawingExport`：ASCII DXF + SVG（含尺寸），无外部 dxflib |
| 持久化 | JSON v2（投影角、视图、尺寸） |

## ABI

- 宿主：`CLOUDSIM_PLUGIN_HOST_VERSION` = `0x00012200`（1.34.0）
- 插件：`minHostVersion` = `1.34.0`

## 编译

Debug\|x64 与 Release\|x64 已编：GeometryAlgorithm → CloudSimPluginSDK → Widget → CloudSimHost → EngineeringDrawing / ProcessFlow / GeometricModeling。
