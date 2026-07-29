# FINAL — 三维模型 → 二维工程图

## 交付摘要

独立插件 `EngineeringDrawingPlugin`（`com.cloudsim.drawing`）已完成一期 MVP、二期增强、三期完善与四期制图可用性：B-rep → OCC HLR 工程图、图幅草图、图框标题栏、尺寸/角度/引线、局部放大、SVG/DXF/PDF 导出，以及宿主模式条 Ribbon。

## 一期要点

- 算法：`geoalgo::projectShapeHlr` / 三视图（`HlrProject.*`，`TKHLR`）
- UI：工程图工作区、`DrawingSheetCanvasWidget`、侧车 `engineeringDrawing`
- 布局：归一化后第一角法网格，可见实线 / 隐藏虚线

## 二期要点

| 能力 | 实现 |
|------|------|
| 通用侧栏 | `enterAlternateSideUi` / `exitAlternateSideUi`（1.34.0） |
| 投影 | `PluginDrawingProjectParams` + `projectBrepToEngineeringDrawing`；第一/第三角 |
| 轴测 / 剖视 | `HlrViewKind::Iso`；中面剖视 |
| 标注 / 拖动 / 局部放大 | 线性尺寸、细节视图、拖视图 |
| 导出 | SVG + ASCII DXF + PDF（`QPdfWriter` / `paintSheet`）；DXF 用户图层名 |
| 图层 | 自定义层、当前层、显示/锁定隔离；JSON **v5** |
| 持久化 | JSON（投影角、视图、尺寸） |

## 三期要点

| 能力 | 实现 |
|------|------|
| 模式 Ribbon | `DrawingRibbonBar` + `setModeToolBar`（视图 / 绘图 / 标注） |
| 图幅草图 | 复用 `SketchDocument2d` / `SheetSketchAdapter`；JSON v3 `sketch` |
| HLR 尺寸捕捉 | 视图折线顶点 + 段中点吸附 |
| 局部倍率 | 页工具栏 `1.5–10`，写入 `contentScale` |
| DXF 图层 | `VISIBLE` / `HIDDEN` / `DIM` / `SKETCH` / `FRAME` |

## 四期要点

| 能力 | 实现 |
|------|------|
| 图框标题栏 | `SheetPaper`（A4/A3、横纵）；页工具栏图幅/图名 |
| 尺寸随视图 | `anchorViewId`；`moveViewBy` 同步 dims/notes |
| 角度 / 文字 | `Kind::Angle`；`SheetNote`；Ribbon + 导出 |
| 用户图层 | 自定义层、显示/锁定、当前层；JSON **v5** |
| 自定义剖切 | `sectionOriginMm` / `sectionNormal`；Algo 任意 `gp_Pln` |
| 细节增强 | Liang-Barsky 裁剪；侧栏改名/倍率/删除 |
| HLR 半径 | 折线圆拟合；失败圆心+圆周手动 |
| 持久化 | `layers` / `currentLayerId` / `layerId` / `sketchLayers` |

## ABI

- 宿主：`CLOUDSIM_PLUGIN_HOST_VERSION` ≥ `0x00012A00`（1.42.0）
- 插件：`minHostVersion` = `1.42.0`

## 编译

Debug|x64 与 Release|x64：`EngineeringDrawingPlugin`（及依赖链按需）。
