# ALIGNMENT — 三维模型 → 二维工程图

## 原始需求

根据三维模型生成二维工程图；二维画布参考工艺流程画布；独立插件。

## 项目上下文

- 三维源：文档内 `BrepModel` / `ParametricBrepModel`（OCC `ShapeHandle`）
- 工艺流程：`ProcessFlowPlugin` 自研 `QWidget`+`QPainter`，经 `setCentralAlternateWidget` / `enterProcessFlowSideUi` / `claimWorkspaceMode` 进入
- 几何插件只链 PluginSDK；OCC 算法在 `GeometryAlgorithm`，经 `IPluginGeometryHost` 暴露
- 仓库内尚无 HLR / 工程图模块

## 边界确认

**一期做：**

- 独立插件 `EngineeringDrawingPlugin`（`com.cloudsim.drawing`）
- 第一角法标准三视图（正/俯/右）+ OCC HLR
- 可见线实线、隐藏线虚线
- 画布缩放/平移/网格/适应窗口
- 工程侧车键 `engineeringDrawing` 持久化

**一期不做：** 尺寸标注、剖视、轴测、DXF/PDF 导出、拖视图、编辑投影线

**不改：** GeometricModelingPlugin 业务菜单与特征链

## 需求理解

- 「参考工艺流程画布」= 复用工作区切换与缩放平移交互模式，**不**复用节点图数据模型
- 出图对象为文档内任意可计算 B-rep，非仅几何建模产物

## 已确认决策

1. 形态：工程图风格三视图 + HLR
2. 落点：独立插件
3. 投影法：第一角法
