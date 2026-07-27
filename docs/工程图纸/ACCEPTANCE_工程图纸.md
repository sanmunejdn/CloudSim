# ACCEPTANCE — 三维模型 → 二维工程图

## 一期验收

| # | 标准 | 状态 |
|---|------|------|
| 1 | Debug\|x64：GeometryAlgorithm / CloudSimPluginSDK / CloudSimHost / EngineeringDrawingPlugin 编译通过 | 通过 |
| 2 | Release\|x64：同上 | 通过 |
| 3 | 产物 `bin/x64d\|x64/plugins/com.cloudsim.drawing/` 含 DLL + plugin.json | 通过 |
| 4 | 宿主 ABI 1.33.0 + `projectBrepHlrToDrawing` | 通过（已演进至 1.34.0） |
| 5 | 侧车键 `engineeringDrawing`；插件 save/load | 通过（代码） |
| 6 | 运行时：生成三视图 / 缩放平移 / 模式互斥 | 待人工点验 |

## 二期验收

| # | 标准 | 状态 |
|---|------|------|
| 7 | 宿主 ABI 1.34.0（`0x00012200`）：`enterAlternateSideUi` / `projectBrepToEngineeringDrawing` | 通过 |
| 8 | 第一角 / 第三角切换 + 等轴测视图生成与布局 | 通过（代码） |
| 9 | 中面剖视图（正/俯/右中面可选） | 通过（代码） |
| 10 | 线性尺寸（两点）+ 手动拖视图 + 局部放大（拖框 ×2） | 通过（代码） |
| 11 | 导出 SVG / ASCII DXF（含尺寸） | 通过（代码） |
| 12 | Debug\|x64 与 Release\|x64 全链编译通过 | 通过 |
| 13 | `minHostVersion` = 1.34.0；旧 `enterProcessFlowSideUi` 仍转发 | 通过 |
| 14 | 运行时：角法/轴测/剖视/标注/拖动/局部放大/导出 | 待人工点验 |

## 产物路径

- Debug：`bin/x64d/plugins/com.cloudsim.drawing/EngineeringDrawingPlugin.dll`
- Release：`bin/x64/plugins/com.cloudsim.drawing/EngineeringDrawingPlugin.dll`
