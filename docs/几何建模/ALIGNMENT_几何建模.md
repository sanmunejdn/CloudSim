# ALIGNMENT — 几何建模 SDK 插件

## 原始需求

开发 SDK 插件「几何建模」：类似 SolidWorks，基于二维约束草图拉伸/裁剪生成 B-rep；记录建模过程（特征树 + Undo）；菜单进入；取消全局左右栏仅保留 AI；中区为真实 3D，选面后在该平面绘制草图。参考项目以复制实现为主。

## 项目理解

- CloudSim：Qt 5.14 + OSG + OCC；插件仅链 `CloudSimPluginSDK`
- 已有 `GeometryPlugin`（离散/求交）、`BrepModel`、`brepBoolean`、面拾取
- 布局可复用 `enterProcessFlowSideUi` / `setCentralAlternateWidget`（工艺流程）
- 无全局 CAD 特征树 / 约束草图 / Pad-Pocket

## 边界确认

| 纳入一期 | 排除 |
|----------|------|
| 独立插件 `com.cloudsim.geomodeling` | 并入 GeometryPlugin |
| 菜单进入/返回；左右隐藏；仅 AI | 工艺流程式左右插件 Dock |
| 中央页嵌入活动文档 3D；选平面面建草图 | 非平面曲面上草图 |
| PlaneGCS 约束（点线圆/弧 + 基础约束） | 椭圆/样条/完整 Sketcher 全约束 |
| Pad/Pocket 定长 + 特征树 rebuild + Undo | UpToFace、圆角、多 Body |
| 复制 FreeCAD planegcs / Extrude 核、OneCAD 模块 | 复制 dune3d/SolveSpace（GPL） |

## 关键假设

1. Host ABI 追加 embed 视口 / face plane / overlay / extrude；版本 bump
2. 最终实体写入文档 `BrepModel`；特征 JSON 经工程钩子持久化
3. 一期以平面 Face 为主；曲面提示不支持
