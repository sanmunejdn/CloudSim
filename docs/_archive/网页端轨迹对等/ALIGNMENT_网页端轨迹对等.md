# ALIGNMENT — 网页端轨迹对等

## 原始需求
网页端完整对等桌面「轨迹生成 / 轨迹编辑」：CAD/BREP 视口拾取、Mesh、PathPlan、全量算子、模板、撤销重做、Raw 预览与 Apply→LINE。

## 项目上下文
- 栈：`CloudSimWeb.exe` + Headless Host + Gateway + `public-fallback` Three.js
- 桌面参照：`TrajectoryEditSession` / `RobotSimulationDockWidget` / `FeatureTrajectoryPageWidget`
- 约束：桌面零回归；Host 旁路 `Headless*`；Debug|x64 + Release|x64

## 边界
- 做：Headless 会话、Gateway API、fallback UI、文档
- 不做：改桌面 Widget 语义；绑运行中的 `CloudSim.exe`；本轮强制 React 壳

## 已确认决策
- 拾取：Three.js 射线 → Host BREP `ShapeRayPick`
- 对等范围：用户选定完整能力面（方案 C）
