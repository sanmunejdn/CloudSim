# ALIGNMENT — 网页端坐标系对齐

## 原始需求

完善网页版「坐标系」页，对齐桌面 `RobotFrameSettingsWidget`（工具/用户系 CRUD、激活、显示、TCP 捕获/重置、3D 叠加轴、工程持久化）。

## 项目上下文

- 栈：`CloudSimWeb.exe` + Headless Host + Gateway + `public-fallback`
- 桌面参照：`RobotFrameSettingsWidget` / `RobotSimulationController::onRobotCoordinateFramesChanged` / `refreshRobotCoordinateFrameOverlays`
- 数据：`RobotCoordinateFrameSet`（`RobotScene`）；Headless 已持有，无 REST
- 约束：桌面零回归；Debug|x64 + Release|x64；产物落 `$(OutDir)web`

## 边界

- 做：Host 捕获/重置/overlay/kinematics 保存；Gateway frames API；public-fallback 面板与轴叠加；6A 文档
- 不做：外轴/通讯页；Vite React 重写；完整 `PlanResultCache`；绑运行中桌面进程

## 已确认决策

- UI 落 `public-fallback`（与轨迹页一致）
- 对齐桌面 Frames 核心能力（含捕获与场景轴）
- Active/几何变更做轻量 instruction tool context 同步，不移植 plan 缓存
