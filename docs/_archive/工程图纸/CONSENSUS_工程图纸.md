# CONSENSUS — 三维模型 → 二维工程图

## 需求描述

用户通过菜单「工程图」进入独立图纸工作区，选择当前文档中的 B-rep，一键生成第一角法三视图（正/俯/右），以 OCC HLR 区分可见/隐藏线，在 2D 图幅画布上查看并可随工程保存。

## 验收标准

1. Debug|x64 与 Release|x64 编译通过（GeometryAlgorithm、PluginSDK、PluginHost、Host、EngineeringDrawingPlugin）
2. 含 B-rep 文档 → 进入工程图 → 选模型 → 生成后中央出现三视图
3. 可见线实线、隐藏线虚线
4. 滚轮缩放、中键或 Alt 平移、适应窗口可用
5. 保存/再打开可恢复图纸（折线缓存或参数可重建）
6. 与工艺流程/几何建模互斥（`claimWorkspaceMode`）

## 技术方案

| 层 | 方案 |
|----|------|
| 算法 | `geoalgo::projectShapeHlr`：`HLRBRep_Algo` → 离散折线（图面 xy） |
| ABI | `IPluginGeometryHost::projectBrepHlrToDrawing`（宿主 ≥ 1.33.0） |
| 插件 | `EngineeringDrawingPlugin`；中央 `DrawingPageWidget` + `DrawingSheetCanvasWidget` |
| 侧栏 | 复用 `enterProcessFlowSideUi`：左=模型列表，右=说明 |
| 持久化 | `backend_type::kProjectKeyEngineeringDrawing` |

## 技术约束

- 插件不直接链接 OCC / GeometryAlgorithm
- 宿主虚函数仅末尾追加；bump `CLOUDSIM_PLUGIN_HOST_VERSION` → `0x00012100`
- GeometryAlgorithm 链接 `TKHLR.lib`

## 任务边界

- 一期无标注/导出/剖视（已完成）
- 二期已纳入：DXF/SVG、轴测、第三角、线性尺寸、剖视、局部放大、拖视图、`enterAlternateSideUi`；PDF 仍后置
- 不修改 GeometricModelingPlugin 功能代码（仅侧栏 API 调用名切换）
