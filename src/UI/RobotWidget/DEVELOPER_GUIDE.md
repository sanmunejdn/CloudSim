# RobotWidget Developer Guide

Robot simulation and device UI live in this x64 DLL (`RobotWidget.dll`, `ROBOTWIDGET_LIB`). Widget keeps `DocumentPage`, `OsgWidget`, and TCP drag teach; orchestration uses host interfaces.

## Build (x64)

| Item | Value |
|------|--------|
| Output | `RobotWidget.dll` |
| Defines | `ROBOTWIDGET_LIB` |
| Links (import lib) | `Data`, `OsgWidgetCore`, `BackendVisual`, `GeometryEngine`, `RobotScene`, `RobotUrdf`, `RobotKinematics`, `RunLogger` + OSG |

与 `Widget.dll` **共享**上述引擎 DLL 运行时实例，不在本 DLL 内重复静态嵌入。

## Layout

| Area | Location |
|------|----------|
| Simulation dock (Instructions / Axis / Frames) | `RobotSimulationDockWidget`, page widgets |
| Orchestration | `RobotSimulationController` |
| Host contracts | `IRobotMainWindowHost`, `IRobotDocumentHost`, `IRobotOsgViewHost` |
| FK / matrix helpers | `RobotSimulationMath` |
| Instruction planning context | `RobotInstructionPlanning` |
| URDF import entry | `RobotUrdfImport::registerUrdfRobot` → host |
| Project JSON (robots/programs) | `RobotProjectIo` |
| Property-panel feasible-axis query | `RobotInstructionPropertyEditor` |

## Widget integration

- `MainWindowRobotHost` implements `IRobotDocumentHost` / `IRobotOsgViewHost` / `IRobotMainWindowHost`.
- `MainWindowUiSetup` creates `RobotSimulationController`, simulation dock, and attaches `m_robotSimTimer` via `attachPlaybackTimer`.
- `MainWindowRobotStubs.cpp` forwards slots (`onSimulationInstructionSelectionChanged`, `onRobotAxisJointAnglesChanged`, TCP teach, etc.) to the controller.
- `MainWindowPropertyPanel` still owns the Qt property browser; the controller calls `refreshInstructionPropertyPanel` on the host.

### Host pitfalls (per-link URDF)

| 项 | 说明 |
|----|------|
| `IRobotDocumentHost::robotBackendManagerForKinematics()` | **必须**转发到 `DocumentPage::robotBackendManagerForKinematics()`。默认基类返回 `nullptr` 会导致 `applyJointAnglesViaLinkBackends` 失败，轴控制/拖动/预览均无场景更新。 |
| `IRobotOsgViewHost` 生命周期 | `osgView()` 在 `currentOsgWidget()` 变化时重建 `OsgViewHost`；勿缓存首次 `OsgWidget*`。 |
| `IRobotDocumentHost` 文档切换 | `document()` 在 `currentPage()` 变化时重建 `DocumentHost`（与 OSG 规则一致）。 |

## Build

- Platform: **x64** only (Debug/Release).
- Output: `bin/x64(d)/RobotWidget.dll`.
- Depends: `RobotScene`, `RobotUrdf`, `RobotKinematics`, `GeometryEngine`, `Data`, `RunLogger`, `OsgWidgetCore`, `BackendVisual`, OSG.

See also [`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) §13–§16 and [`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §6.4.

---

## `RobotSimulationController`

Central orchestration (formerly in `MainWindow.cpp`). Wired in `wireSimulationSignals()` after dock creation.

| 职责 | 入口 |
|------|------|
| 轴控制 | `onRobotAxisJointAnglesChanged` → `applyJointAnglesForInstance` + `requestRedraw` |
| 指令选中预览 | `onSimulationInstructionSelectionChanged` → `applyRobotPoseForInstructionPreview` |
| 添加运动点 | `onSimulationAddInstructionRequested` |
| 运行/停止 | `onSimulationRunRequested` / `onSimulationStopRequested` → `RobotProgramExecutor` |
| TCP 拖动示教 | `onSimulationTcpDragTeachModeChanged` / `onTcpDragTeachPoseChanged` |
| 程序起点 | `captureMotionPreviewProgramStartJoints` → `m_motionPreviewProgramStartJointRad` |
| 坐标系叠加 | `refreshRobotCoordinateFrameOverlays` |

### `wireSimulationSignals`

连接 `SimulationCommandWidget` / `RobotAxisControlWidget` / `RobotFrameSettingsWidget` 信号到 controller 槽。`QTabWidget::currentChanged` 须在 dock 与 `attachPlaybackTimer` 之后连接（见 `MainWindowUiSetup`），避免构造期 `stopRobotSimulation` 空指针。

---

## 示教关节与位姿真源

### `context.currentJointRadCsv`

添加 PTP/LINE 时写入当前实例关节角（rad，逗号分隔）。**预览**与 **Run** 对该指令优先使用示教角，**不再**对该点重算 IK（避免与拖动/捕获姿态不一致）。

| API | 模块 |
|-----|------|
| `RobotInstructionPlanning::encodeJointAnglesRadCsv` | 写入 |
| `RobotInstructionPlanning::jointAnglesRadFromInstructionContext` | 读取 |
| `RobotInstructionPlanning::motionDurationSecFromInstruction` | 段时长（缺省 0.5s） |

### `prepareMotionInstructionForPlanning`

仅设置规划上下文，**禁止**用当前 `rollingQ` 的 FK 覆盖指令 `pose/euler`（`writeTargetTransformToInstruction` 已移除）。写入：

- `context.currentJointRadCsv`（链式种子）
- `context.urdfPath` / `context.tcpLinkName`
- `context.toolFrameMat4`

### 程序起点 `m_motionPreviewProgramStartJointRad`

- 链式预览/Run 的**第一段起点**（与 `initialAngles` 同源）。
- **仅在**该机器人**第一条**运动指令加入程序时更新（`collectMotionInstructions` 条数 ≤ 1）；后续点不再覆盖，避免多点示教时起点被第二点关节顶替。
- 优先从 `m_aggregatedJointAnglesRad` 捕获，否则回退轴滑块。

### TCP 拖动 IK（`applyTcpDragTeachIkFromPose`）

1. `ctx.T_base_target` 来自罗盘（`tcpDragTeachPoseChanged`），并缓存到 `m_lastTcpDragTargetInBase`。
2. `RobotTeachIk::solveTeachIk` → 关节角按 URDF 限位 **钳位**（`clampJointAnglesToInstanceLimits`）。
3. `setJointAnglesRad`（UI 钳位）后 `onRobotAxisJointAnglesChanged(jointAnglesRad())`，保证场景与滑块一致。
4. `updateTcpDragTeachFromTarget` 用钳位后 FK 对齐罗盘（IK 残差时可能略有偏差）。
5. **不**在拖动每帧更新程序起点（仅添加第一条运动指令或空程序结束拖动时可选捕获）。

### 添加指令（`onSimulationAddInstructionRequested`）

| 步骤 | 行为 |
|------|------|
| 位姿 | 若存在 `m_lastTcpDragTargetValid`，用罗盘 `T_base_target` 写 `pose/euler` 与 `writeTargetTransformToInstruction`；否则 `tryCaptureCurrentRobotTcpPose`（关节角优先 `m_aggregatedJointAnglesRad`） |
| 关节上下文 | `context.currentJointRadCsv` = `localJointAnglesForInstance` |
| 添加后 | `captureMotionPreviewProgramStartJoints`（仅首条运动）、`m_skipInstructionPreviewOnce` → 避免立即链式预览把机器人拉离示教姿态 |
| 选中 | `onSimulationInstructionSelectionChanged` 刷新属性与叠加轴 |

### 指令预览（`applyRobotPoseForInstructionPreview`）

自 `motionPreviewProgramStartJointsLocal` 链式处理 `collectMotionInstructions` 至选中点：

- 若该点有 `context.currentJointRadCsv` 且长度 = `nj`：**直接**采用示教关节，跳过 `validate/plan`。
- 否则：`prepareMotionInstructionForPlanning` + `validate` + `plan`，`rollingQ` 取 `plan.jointTargetsRad`。
- 写回 `applyJointAnglesForInstance` 与轴滑块（`m_suppressMotionPreviewStartCapture` 防止误改程序起点）。

### 运行（`onSimulationStartTriggered`）

- `initialAngles` 优先 `m_motionPreviewProgramStartJointRad`，否则当前滑块。
- 对每条运动指令构建 `PlanResult`：有示教 CSV 则 `plan.ok=true`、`jointTargetsRad=示教角`（`plannerName=taughtJointCsv`）；否则走 `RobotInstructionController::plan`。
- `RobotProgramExecutor::tryStart` + `tick` 按段插值 `jointTargetsRad`。

---

## `RobotInstructionPlanning`

| 符号 | 说明 |
|------|------|
| `backupInstructionPose` / `restoreInstructionPose` | 规划前保存/恢复位姿与 extensions |
| `prepareMotionInstructionForPlanning` | 写规划上下文（见上） |
| `encodeJointAnglesRadCsv` / `jointAnglesRadFromInstructionContext` | 示教关节持久化 |
| `motionDurationSecFromInstruction` | 段时长 |

---

## `RobotSimulationMath` / 捕获

`tryCaptureCurrentRobotTcpPose`（controller）：优先 `targetInBaseFromUrdfFlangeFk` + 工具系；关节角优先聚合向量再滑块。per-link 场景见 `captureTcpFromSceneFlangeBackend`。

---

## 页面控件（简要）

| 类 | 说明 |
|----|------|
| `SimulationCommandWidget` | 指令树、Run/Stop、TCP 拖动按钮、`setProgramStore` |
| `RobotAxisControlWidget` | 关节滑块；`setJointAngle` 内 `qBound` 限位 |
| `RobotFrameSettingsWidget` | 工具/用户系；`framesChanged` → 叠加刷新 |
| `InstructionProgramTreeWidget` | `instructionSelected` → 预览 |
| `DevicePageWidget` | URDF 导入 → `onUrdfImportRequested` |

TCP 拖动 OSG 实现仍在 [`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) §13.1。

---

## 扩展指南

1. 仿真 UI/编排：改本 DLL；Widget 扩展 host 与 OSG/TCP。
2. 新增运动点字段：扩展 `RobotInstruction` extensions，并在**预览与 Run** 两条路径一致使用（勿只改其一）。
3. 若新增 `IRobotSimulationDocument` 虚函数，**DocumentHost 必须转发** `DocumentPage` 对应实现。
4. DLL 导出：页面类 `ROBOTWIDGET_EXPORT`（`robotwidget_global.h`）。

---

## 相关文档

- 总架构：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)
- 模块索引：[`../docs/MODULE_DEVELOPER_GUIDES.md`](../docs/MODULE_DEVELOPER_GUIDES.md)
- Widget 宿主 / TCP：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md)
- 指令/执行器：[`../RobotScene/DEVELOPER_GUIDE.md`](../RobotScene/DEVELOPER_GUIDE.md)
- 刚体/工具链：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md)
