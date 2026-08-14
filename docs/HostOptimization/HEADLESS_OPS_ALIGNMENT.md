# HEADLESS_OPS_ALIGNMENT — 桌面 / Web 共路

## 原则

行为落 Host（含 Headless*）；Qt / React 只做皮。禁止只在一侧实现 ops。

## 能力矩阵

| 域 | Host 共路真源 | 桌面入口 | Web 入口 | 对齐状态 |
|----|---------------|----------|----------|----------|
| 文档组合 | `createDocumentHost` / `createHeadlessDocumentHost` | DocumentPage | Gateway 启 Host | 已对齐 |
| 点云 ops | `HeadlessPointCloudBridge` + `DocumentPointCloudOps` | PluginPointCloudHost / Dock | Gateway `/api/pointcloud/*` | 部分：Web 薄、桌面厚；继续把算法调用收进 Bridge |
| 几何 ops | PluginGeometryHost / DocumentGeometryOps | 几何 Dock | `/api/geometry/op` | 部分：优先复用 Host 内 geometry host 实现 |
| 轨迹生成/编辑 | `HeadlessTrajectorySession` | RobotWidget 页 + Controller | React + Gateway 轨迹 API | 高风险分叉；页面向 Session 靠拢（Wave3+） |
| 机器人 FK/URDF | `HeadlessRobotContext` / `IRobotService` | DocumentPage + Controller | HeadlessRobotContext | 已大部分共路 |
| 坐标系 overlay | `RobotCoordinateFrameOps::buildFrameOverlaySnapshot` | Controller overlays | Gateway | 可共路 |
| 工程 IO | `ProjectPackageIo` / `BackendProjectObjectIo` | MainWindow 工程 | Web 工程 API（若有） | 桌面为主 |

## Wave3 落地本轮

1. 固化本表为对齐真源（本文档）。
2. `DocumentHost` 旁路/Follow 外提，避免 Headless 与桌面继续向组合根堆字段。
3. 下一批代码：轨迹页 `document()->backend()` → Session / `documentData()`（见 BACKEND_CALLSITE_INVENTORY §C）。

## 禁止

- Gateway 直链 GeometryAlgorithm / PointCloud 算法 DLL 绕过 Host
- RobotWidget 新增与 `HeadlessTrajectorySession` 平行的第二套会话状态机
