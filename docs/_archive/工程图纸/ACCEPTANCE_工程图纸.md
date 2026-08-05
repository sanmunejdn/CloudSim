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

## 三期验收

| # | 标准 | 状态 |
|---|------|------|
| 15 | 线性尺寸吸附 HLR 折线顶点与段中点 | 通过（代码） |
| 16 | 页工具栏局部倍率 1.5–10，细节视图标题/contentScale 一致 | 通过（代码） |
| 17 | DXF 图层 VISIBLE / HIDDEN / DIM / SKETCH / FRAME | 通过（代码） |
| 18 | Debug\|x64 与 Release\|x64 EngineeringDrawingPlugin 编译通过 | 通过 |
| 19 | 运行时：轮廓尺寸 / 倍率 / DXF 分层 / 存取 | 待人工点验 |

## 四期验收

| # | 标准 | 状态 |
|---|------|------|
| 20 | 图幅纸张 + 标题栏绘制与导出 | 通过（代码） |
| 21 | 拖视图时锚定尺寸/文字随动 | 通过（代码） |
| 22 | 角度尺寸 + 引线文字 + Ribbon | 通过（代码） |
| 23 | JSON v4 存取；Debug/Release 编译 | 通过 |
| 24 | 运行时：图框/随动/角度文字/存取 | 待人工点验 |
| 25 | 导出 PDF（纸张与图幅一致，无网格/交互预览） | 通过（代码） |
| 26 | 用户图层 CRUD + 显示/锁定隔离 | 通过（代码） |
| 27 | JSON v5；DXF 按用户层名导出标注/草图 | 通过（代码） |
| 28 | 运行时：隐藏/锁定/当前层/移层/存取 | 待人工点验 |
| 29 | 自定义剖切平面（原点+法向）经 Host 透传 | 通过（代码） |
| 30 | 细节 Liang-Barsky 边裁剪 + 侧栏管理 | 通过（代码） |
| 31 | HLR 折线圆拟合半径/直径（失败可手动） | 通过（代码） |
| 32 | Host 1.42.0；Debug/Release 编译 | 通过 |

## 产物路径

- Debug：`bin/x64d/plugins/com.cloudsim.drawing/EngineeringDrawingPlugin.dll`
- Release：`bin/x64/plugins/com.cloudsim.drawing/EngineeringDrawingPlugin.dll`
