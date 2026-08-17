# RobotScene 模块开发文档

> **空间契约**：[`../../../docs/spatial_contract_world_pose.md`](../../../docs/spatial_contract_world_pose.md) — per-link FK：`M = M0·inv(T0)·Tq·P`（§8.1）；**P** 与 **M0** 分离，禁止把场景 **W** 写入 **M0**。

## 1. 模块定位

`RobotScene` 承担 **仿真业务逻辑**：机器人指令模型、校验、规划（Planner）、程序执行状态机、将关节角/FK 结果写回文档与 OSG。不依赖 Qt Widget，通过 `IRobotSimulationDocument` / `IRobotBackendPoseSink` 与 `Widget` 解耦。

| 组合依赖 | `GeometryEngine.dll` + `RobotUrdf.dll` + `RobotKinematics.dll` + `Data.dll` + `RunLogger.dll` |
| x64 输出 | `RobotScene.dll` |
| 导出 | `ROBOT_SCENE_API`（x64 构建：`ROBOT_SCENE_LIB`） |

---

## 2. 指令模型（`RobotInstructionModel.h`）

### 2.1 基础类型

| 类型 | 说明 |
|------|------|
| `Vec3` | `x,y,z` — 位姿 mm 或欧拉度 |
| `Type` | `PTP`, `LINE`, `WAIT`, `IF`, `WHILE`, `SET_DO`, `SET_AO` |
| `Category` | `Motion` / `Logic` |

辅助：`categoryForType`, `typeToString`, `typeFromString`。

### 2.2 `class Base`（抽象指令）

| 分组 | 方法 |
|------|------|
| 身份 | `id`, `controllerId`, `type`, `category`, `name` |
| 运动虚属性 | `hasPoseProperty`, `pose`, `setPose`, `eulerDeg`, `speed`/`accel`, `blendRadius`(LINE), `motionAxisConfiguration`, `axisConfig`(legacy string) |
| 逻辑虚属性 | `durationSec`(WAIT), `condition`(IF/WHILE), `ioPort`, `ioBoolValue`, `ioAnalogValue` |
| 嵌套 | `nestedSteps()`, `elseSteps()` — IF/While 子列表 |
| 面板 | `snapshotPropertyRows()`, `applyPropertyChange(key, value, errMsg)` |
| 扩展 | `extensionProperties()`, `setExtensionProperty` — **规划上下文**（见 §5） |

### 2.3 具体指令类

| 类 | 类型 | 关键默认值 |
|----|------|------------|
| `PtpInstruction` | PTP | speed=100, accel=100, `MotionAxisConfiguration` |
| `LineInstruction` | LINE | tcpSpeed=200, tcpAccel=200, blendRadius=0 |
| `WaitInstruction` | WAIT | durationSec=1 |
| `IfInstruction` | IF | `thenSteps`, `elseSteps` + `thenStepsMut()` |
| `WhileInstruction` | WHILE | `bodySteps` |
| `SetDigitalOutputInstruction` | SET_DO | port, bool |
| `SetAnalogOutputInstruction` | SET_AO | port, analog |

`makeInstructionId()` — 生成唯一 id。

---

## 3. 运动轴配置（`RobotInstructionAxisConfiguration.h`）

### 3.1 枚举

| 枚举 | 值 |
|------|-----|
| `ElbowPosture` | Auto, Up, Down |
| `WristPosture` | Auto, NoFlip, Flip |
| `ArmPosture` | Auto, Front, Back |

`kMotionAxisTurnAuto = INT_MIN` — J1/J4/J6 转数不约束。

### 3.2 `struct MotionAxisConfiguration`

| 字段 | 说明 |
|------|------|
| `preset` | AUTO, ELBOW_UP, …, CUSTOM |
| `elbow`, `wrist`, `arm`, `turnJ1/J4/J6` | 分项约束 |

| 方法 | 作用 |
|------|------|
| `resolveEffective(out JointConfigurationClass)` | 展开 preset |
| `isFullyAuto()` | 是否全 AUTO |
| `matchesClass(observed)` | IK 解是否满足构型+转数 |

### 3.3 分类与 JSON

| API | 作用 |
|-----|------|
| `classifyJointConfiguration(q, jointNames, seedQ?)` | 肘/腕/臂/转数观测 |
| `classifyJointTurnRevolutions(q, ref)` | `round((q-ref)/2π)` |
| `solveIkWithAxisConfiguration`（Controller 内） | 多初值 IK + 筛选 |
| `motionAxisConfigurationFromJson` / `writeMotionAxisConfigurationToJson` | 含 `turns.j1/j4/j6` |
| `suggestMotionAxisPresetToken(observed)` | UI 默认 preset |
| `motionAxisConfigurationRequiresConstraint(cfg)` | 显式约束时禁止无约束回退 |
| `formatMotionAxisConfigurationSummary` | 树节点摘要 |

**属性键**（PropertySchema）：`motion.axisConfig.preset`, `.elbow`, `.wrist`, `.arm`, `.turn.j1/j4/j6`。

---

## 4. 条件（`RobotInstructionCondition.h`）

| `ConditionKind` | 字段 |
|-----------------|------|
| `Always` / `Never` | — |
| `Io` | `ioPort`, `ioEquals` |
| `Compare` | `compareLeft`, `compareOp`, `compareRight` |

`conditionFromJson` / `conditionToJson`。

---

## 5. 规划（`RobotInstructionController.h`）

### 5.1 `struct PlanResult`

| 字段 | 说明 |
|------|------|
| `ok` | 是否成功 |
| `plannerName` | 如 `"PtpPlanner"` |
| `summary` | 可读摘要 |
| `durationSec` | 段时长；启用外轴时取 `max(关节梯形, 各外轴梯形：平移≈250mm/s，旋转≈1rad/s)` |
| `hasExternalAxisQ` / `externalAxisQ` | 段末外轴首值（兼容）；完整向量见 `externalAxisQs` |
| `externalAxisQs` | 与配置下标对齐的外轴向量；播放时与段起点逐分量插值 |
| `jointTargetsRad` | 段末关节角 |
| `jointTrajectoryRad` | LINE 笛卡尔采样（URDF）或关节空间回退约 24 点；PTP 常为 `{target}`；**Run 播放须保留**（≥2 点才走轨迹插帧） |

### 5.2 `class PlannerBase`

| 方法 | 说明 |
|------|------|
| `canHandle(Type)` | 是否处理该类型 |
| `validate(cmd, errMsg)` | 规划前校验 |
| `plan(cmd, out, errMsg)` | 输出 `PlanResult` |

**实现类**（`.cpp`，未在头文件导出）：`PtpPlanner`, `LinePlanner`, `ArcPlanner`。

### 5.3 `class Controller`

| 方法 | 说明 |
|------|------|
| `setDhRows` / `clearDhRows` / `hasDhRows` | DH 回退 IK |
| `registerPlanner` / `buildDefaultPlanners()` | 注册 PTP/LINE/ARC |
| `validate` / `plan` | 逻辑指令 → `plannerName="logic"` |
| `queryFeasibleMotionAxisConfigurationOptions(cmd)` | **单次**多初值 IK → 可行 preset/分项 token 列表；由 `RobotSimulationController` 缓存，UI 枚举刷新经后台 Job 调用 |

### 5.4 规划上下文（`extensionProperties` 键）

| 键 | 格式 | 必需场景 |
|----|------|----------|
| `context.currentJointRadCsv` | 逗号分隔 rad | 规划种子；**预览/Run** 有则优先作该点关节目标（见 `RobotWidget` 示教路径） |
| `context.urdfPath` | 绝对路径 | URDF IK、LINE 笛卡尔时长 |
| `context.tcpLinkName` | link 名 | TCP 位姿 IK |
| `legacy.jointIndex` / `legacy.angleDeg` | int / deg | 最后回退 |

### 5.5 IK 解析顺序（运动指令）

1. 有 TCP → URDF 数值 IK（`UrdfNumericalIk` / TeachIk；有欧拉则含姿态）
2. 有 `MotionAxisConfiguration` 约束 → `solveIkWithAxisConfiguration`（无解且显式约束 → **失败，不回退**）
3. DH `ikPositionDampedLeastSquares`（**legacy**，仅 `hasDhRows()`；有 URDF 时生产保持空表）
4. URDF 重试 / legacy 单关节增量

**失败原因（`PlanResult` / `errMsg`）**会区分：
- **主因优先**：缺少上下文、目标过远、未收敛残差、雅可比奇异、轴配置不匹配
- **关节超限**仅在位姿已收敛但解超限时作为主因；未收敛时迭代末越限只作附注（非主因）

---

## 6. 程序工具（`RobotInstructionProgram.h` / `Factory`）

| API | 作用 |
|-----|------|
| `createFromJson` / `toJson` / `createListFromJson` | 序列化 |
| `collectMotionInstructions` / `Recursive` | 规划顺序运动点 |
| `renumberMotionPointIndices` | P1..Pn → `motion.pointIndex` |
| `formatMotionWaypointSummary` | 树显示 |

---

## 7. 执行与回放

### 7.1 `RobotInstructionPlaybackEngine`（遗留）

| 方法 | 说明 |
|------|------|
| `tryStart(doc, osg, queue, initialAngles, err)` | `RobotSimulationCommand` 队列 |
| `tryStartFromPlanResults(...)` | 合并 `PlanResult` |
| `tick(doc, osg)` | ~16ms 插值 |
| `jointAnglesRad()` | 当前角 |

### 7.2 `RobotProgramExecutor`（完整程序）

| 方法 | 说明 |
|------|------|
| `tryStart(doc, osg, io, robotInstanceIndex, program, motionPlanResults, initialAngles, err)` | 含 IF/WHILE/WAIT/IO；`motionPlanResults` 可含 `ok=false` / `lazyPending` 占位 |
| `tick(doc, osg)` | 状态机 + 运动段；遇 `ok=false` 运动段返回 `Aborted`（播到失败点前停止） |
| `updateMotionPlanResult` / `motionPlanResult` | Run 中回写/查询单段规划（懒规划补算） |
| `currentInstruction()` | 当前执行指令：运动中为 `activeMotion()`，否则栈顶 frame 的 `pc-1` 步 |
| `activeMotion()` | 当前运动段（插值中）；规划失败停机时仍指向失败指令 |
| `motionSegmentProgress01()` | 当前运动段进度 [0,1]；非运动中为 1（供外轴插值）；读虚拟时钟 |
| `setPlaybackRate` / `playbackRate` | 播放倍率 `[0.1,10]`；`simElapsed += dtWall * rate`，运行中可改不跳帧；**不改** `plan.durationSec` |
| `lastAbortSummary()` / `abortedDueToFailedPlan()` | 规划失败停机原因 |
| `stop()` / `isRunning()` | 控制（`stop` 不清零倍率） |

私有：`While` 最大迭代 `kMaxWhileIterations = 10000`。

`motionPlanResults` 由 UI 在 **Run 启动时** 急算前缀段并对其余填 `lazyPending`（或失败占位）；播放中由 Widget `ensurePlaybackPlansReady` / lookahead 经 `updateMotionPlanResult` 补齐。任一点规划失败时写入 `ok=false`，**至少一段成功则仍 `tryStart`**，播放至失败点前停止。`lazyPending` 须在进入该段前被消掉，否则视为停机。指令树选中预览在 Widget 层 **单次** `plan`、**不**经过本 executor。Run 期间 Widget 用 `currentInstruction()` 高亮指令树。预览与 Run 的差异见 [`../RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md) §指令树点击预览 vs 仿真运行。

播放插值：`jointTrajectoryRad.size() >= 2` 时优先按轨迹（含段起点）插值；否则对 `jointTargetsRad` 起止 lerp。段进度与 WAIT 均用虚拟时钟（`m_simElapsedSec`）。外轴不在 Executor 内驱动，由 Widget tick 用 `motionSegmentProgress01()` 对 `externalAxisQs`（兼容标量）插值后写文档 Q 再 FK。

### 7.3 `IRobotIoSink`

| 方法 | 说明 |
|------|------|
| `setDigitalOutput` / `setAnalogOutput` | 纯虚 |
| `getDigitalInput` | 默认 false |

`SimulationLogIoSink`（Widget）为内存实现。

---

## 8. 场景运动学（`RobotSceneKinematics.h`）

### 8.1 `struct RobotPerLinkKinematicsSlice`

| 字段 | 说明 |
|------|------|
| `urdfAbsolutePath` | URDF 路径 |
| `sceneRootBackendId` | robot root backend id |
| `linkNameToBackendId` | link → mesh backend |
| `fkMeshWorldT0` | bind 时各 link 网格世界矩阵 **T0**（URDF FK，q=bind） |
| `outerWorldAtBindByBackendId` | bind 时 outer 参考 **M0**（**不含**后续基座位移 **P**） |
| `robotBasePlacementWorld` | 场景根位姿 **P**（对象 gizmo 移动整机关节链时只改此项） |
| `meshVerticesInLinkFrame` | 顶点是否在连杆系 |

**FK 约定（per-link flat OSG）**：

```text
M_link = M0 · inv(T0) · Tq · P
```

- **M0** / **T0**：导入 bind 时冻结；关节示教、TCP IK 后仅改 **Tq**。
- **P**：`DocumentPage::setRobotBasePlacementWorldForInstance`；对象 gizmo 移动整机关节链时由 `applyPerLinkRobotFkFromGizmoAnchor` 反解并 FK（见 Widget §6.3.2）。勿把拖动后的世界矩阵 **W** 直接写入 **M0**（否则 `applyJointAnglesViaLinkBackends` 会双乘 **P**，连杆散开）。
- 从场景恢复 **M0**：`M0 = W · inv(P) · inv(Tq) · T0`（`DocumentPage::reconcilePerLinkOuterBindFromScene`）。

### 8.2 `namespace RobotSceneKinematics`

| 函数 | 作用 |
|------|------|
| `applyJointAnglesFromDocument(doc, osg, anglesRad)` | 按实例循环 per-link / 层级 |
| `applyJointAnglesForInstance(doc, osg, instanceIndex, local, aggregated)` | 单机 |
| `applyJointAnglesViaLinkBackends(doc, osg, mgr, angles, slice)` | `Mnew = M0 * inv(T0) * Tq * P` 写 outer + `setWorldMatrix` |
| `applyPerLinkRobotBasePlacement(osg, mgr, slice, q, P)` | 仅改 **P** 后 FK 全连杆 |
| `computeBasePlacementFromAnchorLinkWorld(slice, anchorId, q, W_anchor, outP)` | gizmo 锚点世界 **W** 反解 **P** |
| `applyMeshWorldMatricesRelativeToBind(...)` | 预计算 FK 相对 bind 更新 |

**调试**：`ROBOT_KINEMATICS_DEBUG=1` → `[RobotKinematicsDBG]` 日志。

---

## 8.3 坐标系（`RobotCoordinateFrames.h`）

### 术语（避免 `tcp` 歧义）

| 符号 | 含义 |
|------|------|
| **`T_base_target`** | 指令 `pose`/`euler`：该点 `motion.tool.frameId` 对应**工具系原点**在基座下的位姿 |
| `T_flange_tool` | 法兰 → 工具（标定）；`positionMm` 在 **法兰连杆轴**（mm），非基座轴；≈**I** 时 `T_base_target` 即法兰中心 |
| `T_base_flange` | 前置变换后送入 IK 的**法兰连杆**目标 |

遗留名 `T_base_tcp` / `tcpInBaseFromPose` 与 `T_base_target` 同义；新代码用 `targetInBaseFromPose`。

### 坐标变换前置（单一 IK，不按工具分叉）

1. 读指令 → `T_base_target`
2. 读工具 → `T_flange_tool`（`motion.tool.frameId` / `context.toolFrameMat4`）
3. **前置**：`T_base_flange = flangeTargetFromToolOriginInBase(T_base_target, T_flange_tool)`
4. IK 只求法兰连杆位姿；禁止在规划器内再乘 `T_tool`

| 类型 | 说明 |
|------|------|
| `RobotRigidFrame` | 平移 mm + 欧拉 deg（`BackendFollowMath` 同约定） |
| `RobotToolFrame` | `id` / `name` / `T_flange_tool` / `flangeLinkName` / **`showInScene`**（默认 `true`） |
| `RobotUserFrame` | `id` / `name` / `T_base_user` / **`showInScene`**（默认 `true`） |
| `RobotCoordinateFrameSet` | `toolFrames[]`、`activeToolFrameId`、`flangeLinkName`、用户系列表、激活 id、全局 3D 显示开关 `showToolFrameInScene` / `showUserFramesInScene` |

**JSON**（`writeCoordinateFrameSetToJson` / `readCoordinateFrameSetFromJson`）：全局键 `showToolFrame`、`showUserFrames`；每帧可选 `showInScene`（缺省 `true`，兼容旧工程）。

| API | 作用 |
|-----|------|
| `targetInBaseFromPose` | `pose` → `T_base_target` |
| `engine::RigidTransform` / `ToolKinematics` | **规范刚体真值**（GeometryEngine + Eigen；四元数存储，mm） |
| `rigidTransformFromFrame` / `frameToMat4` | `RobotRigidFrame` ↔ `RigidTransform` / `BackendMat4` |
| `flangeTargetFromToolOriginInBase` | 委托 `engine::flangeFromToolOrigin`（`composeColumn`） |
| `targetInBaseFromFlange` | 委托 `engine::toolOriginFromFlange`（`composeColumn`） |
| `RobotInstruction::readTargetTransformFromInstruction` | 指令真值（优先 `context.targetTransformQuatCsv`） |
| `RobotMatrixOsg::*` | 遗留 OSG/BackendMat4 薄适配，新逻辑勿再扩展 |
| `flangeTargetFromBaseTcpAndTool` / `tcpInBaseFromPose` | 兼容别名 |
| `tcpInUserFromBaseTcp` / `tcpInBaseFromUserTcp` | 用户系示教换算 |
| `resolveToolFrameForExtension` / `toolMat4ForExtension` | 按点工具 → `context.toolFrameMat4` |
| `resolveUserFrameForExtension` | 用户系 |

### 8.3.1 外部轴（`RobotExternalAxes.h`）

| 类型 | 说明 |
|------|------|
| `RobotExternalAxisConfig` | `motionType` Translate/Rotate、`attachment` RobotBase/Workpiece、`axis[]`、`originMm[]`、`boundBackendId`（工件必填）、可选 `workingFrameId`、行程 |
| `RobotExternalAxisConfigSet` | `axes[]`；门禁 `hasEnabledExternalAxes` / `validateExternalAxisConfigSet` |

持久化：`robotKinematicsInstances[].externalAxes`。联动搜索：`ExternalAxisSearchService` + `TeachIk` 多轴 DOF；未启用时 `ExternalAxisSearch` Op 为 no-op。

**存储契约**：`basePlacementWorld` = P0；运行态 `externalAxisQ[]`（兼容 `externalAxisQMm`）；工件零位 `workpieceBasePlacementWorld[backendId]=W0`；工作架偏置 `workpieceWorkingFrameOffsetByBackend`（W0 局部）。FK：`composeBasePlacementWithExternalAxis` → `P_eff`；`composeWorkpiecePlacementWithExternalAxis` → `W_eff`；`composeWorkpieceWorkingFrameInRobotP0` → `T_p0_work`。Mat4 平移在 `[3,7,11]`。专题：[`docs/外部轴类型拓宽/`](../../../docs/_archive/外部轴类型拓宽/)、[`docs/外部轴联动求解/`](../../../docs/_archive/外部轴联动求解/)。

**REP（启用 Workpiece）**：示教/规划 TCP 相对工作架 `T_work`；外层采样工件轴 → `T_p0_goal = T_p0_work(q_w)*T_work`；内层仅 RobotBase TeachIk。Host 经 `Controller::WorkpieceIkFrameContext`（及 `PlanJobPayload`）注入 P0/W0/Offset。

**指令扩展键（PTP/LINE）**

| 键 | 说明 |
|----|------|
| `context.externalAxisQCsv` | 多轴外轴量（优先） |
| `context.externalAxisQMm` | 首轴兼容 |
| `context.workingTcpTransMmCsv` / `context.workingTcpQuatCsv` | 相对工作架 TCP（REP；缺省则由 `T_p0`+种子 `q_w` 反推） |
| `motion.tool.frameId` | 工具系 id；切换时 **保持 TCP 空间位置**（`pose` 不变），重算 IK |
| `motion.user.frameId` | 用户系 id |
| `motion.target.frame` | `base` / `user` — 面板显示系 |
| `context.toolFrameMat4` | 规划用工具矩阵（按点冻结） |
| `context.poseFrame` | `base_tool_origin`（兼容 `base_tcp`） |
| `context.targetTransformQuatCsv` / `context.targetTransformTransMmCsv` | 指令真值刚体（`readTargetTransformFromInstruction` 优先读取） |
| `context.flangeLinkName` / `context.tcpLinkName` | 法兰 link（IK/FK） |

`[IK残差]` 分列：`toolOrigin`、`flangeTarget`、`fkFlange`、`fkToolOrigin`；主指标 **`residualTcpMm`** = \|toolOrigin − fkToolOrigin\|（FK 经 `toolOriginFromFlange`）。

规划上下文：按点写入工具矩阵与法兰 link。指令 `pose` = **基座下工具原点**；3D 轴与绿/红可达性同前。

### 规划真值一致性（Pose vs TargetTransform）

- `pose/euler` 与 `context.targetTransform*` 必须保持同源一致；`ProgramEditCommand` 对位姿做变换时通过 `writeTargetTransformToInstruction` 同步两者。
- 轨迹编辑预览会临时写入 `context.targetTransform*`。当上层（RobotWidget）执行快照恢复时，若快照里不存在该键，必须显式清理旧键后再恢复扩展属性。
- 否则会出现“`pose` 已回退但 `readTargetTransformFromInstruction` 仍是旧值”的状态，Apply 会基于错误真值再次增量，表现为位置被重复作用。

### 工具链 FK（禁止误用 OSG 裸乘）

1. `T_base_flange`：`UrdfRobotLoader::computeLinkWorldMatrices` → `engine::rigidTransformFromOsg(linkWorld[flange])`。
2. `T_flange_tool`：`rigidTransformFromFrame` 或 `rigidTransformFromBackendMat4(context.toolFrameMat4)`。
3. `T_base_target = engine::toolOriginFromFlange(T_base_flange, T_flange_tool)` — **必须**走 `ToolKinematics`（内部 `composeColumn`）。
4. IK 前置：`flangeFromToolOrigin(T_base_target, T_flange_tool)`（`ikLinkTargetFromInstruction`）。

**反例（已修复）**：`linkWorld * osgMatrixFromBackendMat4(T_tool)` 或 `composeScene` 组合 URDF 法兰与 Eigen 工具 → 法兰系 `(0,0,-200)` 可能表现为仅基座 Z 变化。`RobotMatrixOsg::targetInBaseFromFlangeLinkWorld` 与 `Widget::osgTcpInBaseFromFlangeLinkWorld` 已改为 engine 路径。

**验收**：法兰 Ry≈90° 时，工具 Z=-200 应在基座 X（或 Y）体现约 200 mm 偏移，而非仅 ΔZ。见 `GeometryEngine::runSelfTest`。

## 8.4 程序导出

### Canonical v1（主路径，`RobotCanonicalProgramExport.h`）

| 字段 | 说明 |
|------|------|
| `format` | `cloudsim.program_export`，`schemaVersion: 1` |
| `exportLayout` | 默认 `nested_tree`（IF `then`/`else`，WHILE `body`） |
| `instructions[]` | 与 `program.steps` 同构的运行记录 |
| `flatMotionSequence` | DFS 运动叶索引，与仿真顺序一致 |
| `coordinateFrames` | 完整 tool/user 帧定义 |

仿真 **Export…** 写 Canonical 临时文件，再经 RobotWidget `PythonScriptCaller` 调用 `resource/Python/ExportPython/*Export.py` 生成品牌程序（用户对话框选择最终路径）。离线 stub 仍见 `CloudSim/src/UI/RobotWidget/tools/robot_postprocess/`；正式路径以 resource + pybind 为准。详见 [`docs/机器人程序品牌导出/`](../../../docs/_archive/机器人程序品牌导出/)。

### 遗留（`RobotProgramExport.h`）

| API | 作用 |
|-----|------|
| `buildExportResult` | 扁平运动点 + 关节角（过渡） |

## 8.5 PathPlan 指令（`Type::PathPlan`，`Category::Planning`）

- 字段：`pipeline[]`、`appliedHistory[]`、`phase`、`outputGroupId`、`rawTrajectoryKey`、`sourceFeatureJson`
- `sourceFeatureJson`：持久化 `FeatureListDocument`（工件 backend、特征列表、每行 `strategyId` + `params` + 几何索引）
- Raw 存 `RobotProgramCatalog::pathPlanRaws`（工程 JSON `pathPlanRaws`）
- 执行器/仿真/Canonical 导出跳过 `Category::Planning`；Apply 输出分组 `role=path_plan_output`
- 编辑命令：`InsertPathPlanCommand`、`RemovePathPlanCommand`、`UpdatePathPlanPipelineCommand`、`UpdatePathPlanRawCommand`、`UpdatePathPlanApplyStateCommand`；轨迹 Apply 用 `CompositeProgramEditCommand` 打包程序替换与 PathPlan 状态
- UI：`TrajectoryGenerationPageWidget` 顶栏绑定 PathPlan；`beginEditBoundPathPlan` + `reloadBoundPathPlanFromStore` 一次恢复特征/离散参数/算子；详见 [`RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md) §PathPlan 持久化与「开始修改」
- `unifiedTrajectoryMergeIntoProgram` / `emitRawTrajectoryToProgram(..., pathPlanInstructionId)`：仅删除并替换当前 PathPlan 的 `PathPlanOutput` 成员路点，其它 PathPlan 输出分组与运动路点保留
- 旧工程加载：`migrateLegacyPathPlans`（无 PathPlan 但有运动/分组时补默认项）

---

## 9. 文档接口

### 9.1 `IRobotSimulationDocument`

| 方法 | 说明 |
|------|------|
| `robotKinematicInstanceCount()` | 多机数量 |
| `robotUrdfAbsolutePathForInstance(i)` | 每机 URDF |
| `robotJointKeyPrefixForInstance(i)` | 关节键前缀 |
| `robotUsesPerLinkBackendsForInstance(i)` | 是否 per-link |
| `robotPerLinkKinematicsForInstance(i, out)` | 切片数据 |
| `robotJointMatrixTransform(jointName)` | 层级模式关节 MT |
| `robotLinkNameToBackendId()` | 聚合 map（兼容 UI） |
| `notifyRobotKinematicsAppliedToScene()` | 跟随脏标记 |

由 `DocumentPage` 实现。

### 9.2 `IRobotBackendPoseSink`

| 方法 | 说明 |
|------|------|
| `get/setBackendRootWorldMatrix` | 世界/局部 |
| `tryGetBackendModelCenterMm` | 可选 |
| `syncRobotMeshBackendPoseAfterKinematics(mesh)` | FK 后同步 PAT |

由 `OsgWidget` / `BackendSceneDocumentFacade::poseSink()` 实现。

---

## 10. 属性 Schema（`RobotInstructionPropertySchema.h`）

| 函数 | objectTypeId |
|------|----------------|
| `ptpInstructionPropertySchema()` | `robot_instruction.ptp` |
| `lineInstructionPropertySchema()` | `robot_instruction.line` |
| `waitInstructionPropertySchema()` | `robot_instruction.wait` |
| `setDoInstructionPropertySchema()` | `robot_instruction.set_do` |
| `setAoInstructionPropertySchema()` | `robot_instruction.set_ao` |
| `schemaForInstructionType(Type)` | 分发 |

---

## 11. 典型工作流

```mermaid
flowchart LR
  A[编辑指令树] --> B[设置 extension context]
  B --> C[Controller.plan 每条运动]
  C --> D[RobotProgramExecutor.tryStart]
  D --> E[RobotSceneKinematics.apply*]
  E --> F[IRobotBackendPoseSink]
```

**预览**（非运行）：`applyRobotPoseForInstructionPreview` 经链式种子对选中点单次 `plan`（或 `shouldUseTaughtJointCsv` + 残差门控）。**坐标系页**添加未激活工具系不触发全程序 reachability；切换激活工具会同步 `active` 路点 context 并失效示教关节。轴配置可行列表经 `queryFeasibleMotionAxisConfigurationOptions`（`RobotSimulationController` 缓存 + **后台 Job** 刷新枚举，见 [`../RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md)）。

**运行**：`onSimulationStartTriggered` 急算前缀 `PlanResult`（其余 `lazyPending`）；播放中 `ensurePlaybackPlansReady` / `tickLookaheadPlanning` 补齐；任一点失败则播到该点前 `Aborted`；tick 内 `currentInstruction()` 驱动指令树高亮。程序起点仅在**第一条**运动指令加入时更新（见 [`../RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md)）。

**末端拖动示教**（非运行、不写指令）：进入前 per-link 调用 `reconcilePerLinkOuterBindFromScene`；屏幕空间平移更新 `T_base_target` → `RobotTeachIk` → 关节钳位 → `applyJointAnglesForInstance`；基座世界取 **P**（`robotBaseWorldMatrixForInstance`）。添加指令时用罗盘位姿 + `currentJointRadCsv` 落盘。见 [`../Widget/DEVELOPER_GUIDE.md`](../../UI/Widget/DEVELOPER_GUIDE.md) §13.1、§6.3.2（**M0**/**P**）、[`../RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md)。

### 11.1 `RobotTeachIk`

示教策略层：外轴 bake/unbake、联立多候选代价。**臂位姿 DLS** 在 `RobotUrdf::solveArmPoseDampedLeastSquares`（`UrdfIkSolverOptions` / Workspace）。

| API | 说明 |
|-----|------|
| `TeachIkContext` | `urdfPath`、`ikLinkName`、`T_base_target`、`seedJointRad`、`T_flange_tool`、`options` |
| `solveTeachIk` | 交互示教 IK；法兰目标经 `engine::flangeFromToolOrigin` |
| `solveTeachIkCoordinatedDrag` | 拖动联立多候选 |

架构图：[`../../../docs/_archive/robot-kinematics-workspace/diagrams/target-architecture.html`](../../../docs/_archive/robot-kinematics-workspace/diagrams/target-architecture.html) · 热路径：[drag-hotpath-dataflow.html](../../../docs/_archive/robot-kinematics-workspace/diagrams/drag-hotpath-dataflow.html)

DH：`setDhRows` 仅 **无 URDF** legacy；有 URDF 时保持 `clearDhRows`。

---

## 12. 多程序与分组（`RobotProgramCatalog`）

每台机器人实例对应一份 `RobotProgramCatalog`（`RobotProgramStore::catalogFor`）。JSON v4 含 `programs[]` 与 `groups[]`；旧工程兼容见 `RobotProgramJsonIo`。

| 类型 | 说明 |
|------|------|
| `RobotProgram` | `id` / `name` / `steps` / `groups` |
| `InstructionGroup` | 元数据分组；`memberInstructionIds` 指向根层级任意指令 id（PTP/LINE/WAIT/IF/WHILE/IO 等）；**不改变** `steps` 执行顺序 |

指令页在 `InstructionProgramTreeWidget` 中以 `NodeKind::Group` 嵌套显示；创建/解散/重命名经树右键 + `ProgramEditService`。轨迹编辑页顶栏分组下拉仅用于 `OpScope::Group` 选择。
| `kDefaultMainProgramId` | `"main"` |

| API | 作用 |
|-----|------|
| `activeSteps()` | 当前活动程序的 `steps` 向量（与 `RobotProgramStore::activeProgram()` 同源） |
| `resolveGroupMembers` | 分组 → 运动路点指针（过滤 `isMotionWaypointType`） |
| `expandToMotionWaypointIds` | 顶层成员 id 列表 → 递归展开 IF/WHILE 子树内全部运动路点 id（轨迹平移/旋转 scope） |
| `resolveOpScopeInstructionIds` | 轨迹块 `OpScope` → 指令 id 列表（Apply/Preview 共用） |
| `pruneGroupMembers` | 删除指令后清理分组引用 |

### `OpScope`（`TrajectoryPipelineTypes.h`）

| `OpScope::Kind` | 解析规则 |
|-----------------|----------|
| `EntireProgram` | `collectMotionInstructions` 全部 PTP/LINE/ARC |
| `Group` | 匹配 `scope.groupId` 的 `memberInstructionIds`（**不**过滤运动类型；若分组不存在则返回空列表，由 UI `reconcilePipelineScopes` 协调） |
| `PointIndexRange` | `motionPointIndex` ∈ [pointFrom, pointTo]（扩展键 `motion.pointIndex`） |
| `InstructionIds` | 显式 id 列表 |

---

## 13. 轨迹流水线与程序编辑 Command

### 13.0 算法库与桥接

| 工程 | 说明 |
|------|------|
| [`TrajectoryAlgorithm`](../TrajectoryAlgorithm/DEVELOPER_GUIDE.md) | `ITrajectoryOp`、Registry、ConfigRegistry、ParamSchema、Codec、`TrajectoryTransformMath` |
| [`TrajectoryAlgorithmBuiltins`](../TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md) | 18 种原子块（Translate … ExternalAxisSearch）；共享 `UnifiedTrajectoryPathMath` |
| [`TrajectoryOpBridge.h`](inc/TrajectoryOpBridge.h) | **UI 唯一入口**：Registry、参数读写、模板 JSON（避免 RobotWidget 重复链接静态库） |
| [`TrajectoryPipelineEngine.h`](inc/TrajectoryPipelineEngine.h) | 唯一管道执行器：`Ingress → processPath × N → Egress` |

`ensureTrajectoryOpBuiltinsRegistered()` 在引擎执行与 UI 构造时调用。Apply/预览均经 `TrajectoryEditSession` + `TrajectoryPipelineEngine::executeFull`。

### 13.1 流水线描述符

| 类型 | 说明 |
|------|------|
| `TrajectoryOpKind` | Translate / Rotate / Mirror / Delete / Duplicate / Reorder / Approach / Retract / Resample / OffsetAlongNormal / OffsetLateral / SmoothPose / AssignBlend / AssignSpeedZone / Weave / ReachabilityFilter / ExternalAxisSearch / ProjectToGeometry / NonRigidRegistration |
| `TrajectoryOpDescriptor` | `kind` + `OpScope` + `params` + `enabled`（未启用则引擎跳过） |
| `TransformReferenceFrame` | `World` / `Body`（`TranslateParams` / `RotateParams` 的 `frame`） |
| `TrajectoryPipelineEngine` | `UnifiedTrajectory` 管道 IR；Session 持有并驱动预览/Apply（`executeFull` / `executeFrom`） |
| `ProcessFlowPresets.json` | 工艺预设（焊缝/涂胶/打磨）展开为原子块 `pipeline` 列表 |

工艺模板：`buildRecipePreset(Weld/Glue/Grind)` → `ProcessFlowPresetLoader` 读 `ProcessFlowPresets.json` 原子 `ops` / `pipeline`；不再支持 `RecipeWeld` 等复合 kind token。

### 13.2 `ProgramEditCommand` / `ProgramEditStack`

| Command（示例） | 作用 |
|-----------------|------|
| `TransformMotionSegmentCommand` | 平移/旋转 scope 内路点 |
| `InsertInstructionCommand` / `RemoveInstructionCommand` | 增删（Duplicate 等） |
| `CreateInstructionGroupCommand` | 创建分组 |
| `RemoveInstructionGroupCommand` | 解散分组（不删指令） |
| `RenameInstructionGroupCommand` | 重命名分组 |

`InstructionProgramDocument`：在 `activeProgram()` 步骤树上按 id 查找/修改。Command 使用 `shared_ptr` 跨 DLL 边界（`ProgramEditStack::CommandPtr`）。

Apply 统一走 **Unified IR + 引擎**（`TrajectoryEditSession::apply`）。

| 条件 | Apply |
|------|--------|
| `m_rawTrajectory` 存在 | 引擎 `setUsingRaw(true)`：`ingressUnifiedFromRaw` → `pendingPreRaw` → `committed(accumulated)` → `draft(本批)` → `unifiedTrajectoryToProgram` |
| 无 raw，程序有路点 | `ingressUnifiedFromProgram` → `committed` → `draft` → materialize |
| 无 raw 且无路点 | 报错「无原始轨迹且程序中无路点」 |

多次 Apply 从**同一 CAD raw** 重放 `m_accumulatedGeometryOps`（committed），不以 `m_bakedWorldRaw` 为起点。预览与 Apply 共用 `TrajectoryPipelineEngine::executeFull`；有 raw 时 OSG 用 `applyWorldRawTrajectoryPreviewToOsg`；无 raw 且含拓扑变更块时用 OSG 叠加层（见 RobotWidget §预览分支）。

UI 侧：Apply 成功后禁用「生成程序」、`onRawEmitProgram` 硬门禁；清除 raw 叠加层并刷新指令路点轴。

撤销/重做后流水线 scope 与程序分组可能不一致（例如撤销「创建分组」）；UI 层 `TrajectoryEditPageWidget::reconcilePipelineScopes` 在 Preview/Apply 与 `revisionChanged` 时回退失效的 `Group` scope，详见 RobotWidget §轨迹编辑。

业务编排与 UI 绑定见 [`../RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md) §轨迹编辑。

---

## 14. CAD 轨迹中间表示（`RawTrajectory.h`）

特征→轨迹基于 `RawTrajectory`，几何编辑在 `UnifiedTrajectory` 上由原子块管道完成：

```text
FeatureSpec → discretizeFeature → RawPath → importRawPathToTrajectory → RawTrajectory
  → Ingress → TrajectoryPipelineEngine（原子块 processPath）→ Egress → RobotProgram
```

| 类型 | 说明 |
|------|------|
| `RawTrajectory` | `points` + `TrajectoryContext` + 溯源 `sourceFeature`；可选 `segmentEndExclusive`（Mesh 截面多交线段） |
| `TrajectoryPoint` | `poseMm`、`eulerDeg`、`blendRadiusMm`、`speedMmPerSec`、`reachable` |
| `TrajectoryContext` | `workpieceFrameId`、`toolFrameId`、`externalAxes[]`（地轨/变位机快照，非工艺字段） |
| `RawTrajectoryOpKind` | **遗留枚举**；新代码使用 `TrajectoryOpKind` + `ITrajectoryOp::processPath` |
| `UnifiedTrajectory` | 管道 IR：pose/euler/blend/speed/reachable + `sourceInstructionId` |

| API | 作用 |
|-----|------|
| `importRawPathToTrajectory` | `RawPath` + `FrameStrategy`（法向 Z / 固定 Z / 切向 X）→ 姿态；复制 `segmentEndExclusive` |
| `importMeshRawPathToRawTrajectory` | Mesh 轨迹：`generateRawPath` → `RawTrajectory`（见 MeshTrajectorySDK） |
| `importTubularGrindingPointsToRawTrajectory` | **预留（桩）**：`geoalgo::TubularGrindingProjectedPoints` → `RawTrajectory`；见 [`inc/TubularGrindingTrajectoryIngress.h`](inc/TubularGrindingTrajectoryIngress.h) |
| `ingressUnifiedFromRaw` / `ingressUnifiedFromProgram` | Raw 或程序 → Unified（引擎 Ingress） |
| `buildRecipePreset` | 工艺 UI 入口 → `ProcessFlowPresets.json` 原子 `pipeline` |
| `unifiedTrajectoryFromRaw` / `unifiedTrajectoryFromProgram` | 同上 Ingress 的薄封装 |
| `unifiedTrajectoryToRaw` | Unified → 点位列（轨迹编辑预览为世界 mm，由 UI 直接画 OSG） |
| `RobotSceneGeometryProjection` | `IGeometryProjection` 适配，封装 `projectUnifiedToGeometry` |
| `RobotSceneNonRigidTrajectoryWarp` | 绑定与 SPARE 在**源模型坐标系**：轨迹经当前 `Ts` 反变换后绑定模型网格；目标经当前 `Tt→Ts` 表达相对位姿；写回再乘 `Ts`。避免源/目标 gizmo 移动后仍用旧世界坐标绑定 |
| `emitRawTrajectoryToProgram` | 可达点 → `LineInstruction`；多段时按 `segmentEndExclusive` 建多个输出分组（`*_S1`…）；PathPlan 绑定行为同前 |
| `rawTrajectoryToPreviewPolylineXyz` / `rawTrajectoryReachabilityColorsJson` | UI/OSG 预览 |

实现：[`source/RawTrajectory.cpp`](source/RawTrajectory.cpp)。

**与 `TrajectoryAlgorithm` 的关系**：全部 `TrajectoryOpKind` 由 Builtins 实现 `processPath`；Session/Builder 仅编排 Ingress、引擎重放与 Egress，不再维护 Pose 预览链或 Recipe 复合块。

**Phase 说明**：`ReachabilityFilter` 经 `ITrajectoryReachabilityProbe`（`TrajectoryReachabilityProbeService` + TeachIk）写 `reachable`；未注入时 processPath 失败。`ExternalAxisSearch` 已接入对象外轴配置门禁与 `ExternalAxisSearchService`（需 UI 管道注入 URDF/种子）。

---

## 15. 相关文档

- 轨迹框架：[`../TrajectoryAlgorithm/DEVELOPER_GUIDE.md`](../TrajectoryAlgorithm/DEVELOPER_GUIDE.md)
- 内置原子块：[`../TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md`](../TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md)
- 刚体/工具链：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../../Geometry/GeometryEngine/DEVELOPER_GUIDE.md) · [`../GeometryEngine/CONVENTIONS.md`](../../Geometry/GeometryEngine/CONVENTIONS.md)
- URDF：[`../RobotUrdf/DEVELOPER_GUIDE.md`](../RobotUrdf/DEVELOPER_GUIDE.md)
- DH：[`../RobotKinematics/DEVELOPER_GUIDE.md`](../RobotKinematics/DEVELOPER_GUIDE.md)
- 特征离散：[`../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.1
- UI 轨迹生成：[`../RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md) §CAD 轨迹生成
- 轴配置详解：[文档索引](../../../docs/README.md) §4.8.1–4.8.3
