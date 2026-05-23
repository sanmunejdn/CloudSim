# RobotScene 模块开发文档

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
| `durationSec` | 段时长 |
| `jointTargetsRad` | 段末关节角 |
| `jointTrajectoryRad` | LINE 约 24 点插值；PTP 常为 `{target}` |

### 5.2 `class PlannerBase`

| 方法 | 说明 |
|------|------|
| `canHandle(Type)` | 是否处理该类型 |
| `validate(cmd, errMsg)` | 规划前校验 |
| `plan(cmd, out, errMsg)` | 输出 `PlanResult` |

**实现类**（`.cpp`，未在头文件导出）：`PtpPlanner`, `LinePlanner`。

### 5.3 `class Controller`

| 方法 | 说明 |
|------|------|
| `setDhRows` / `clearDhRows` / `hasDhRows` | DH 回退 IK |
| `registerPlanner` / `buildDefaultPlanners()` | 注册 PTP/LINE |
| `validate` / `plan` | 逻辑指令 → `plannerName="logic"` |
| `queryFeasibleMotionAxisConfigurationOptions(cmd)` | **单次**多初值 IK → 可行 preset/分项 token 列表 |

### 5.4 规划上下文（`extensionProperties` 键）

| 键 | 格式 | 必需场景 |
|----|------|----------|
| `context.currentJointRadCsv` | 逗号分隔 rad | 规划种子；**预览/Run** 有则优先作该点关节目标（见 `RobotWidget` 示教路径） |
| `context.urdfPath` | 绝对路径 | URDF IK、LINE 笛卡尔时长 |
| `context.tcpLinkName` | link 名 | TCP 位姿 IK |
| `legacy.jointIndex` / `legacy.angleDeg` | int / deg | 最后回退 |

### 5.5 IK 解析顺序（运动指令）

1. 有 TCP → URDF 数值 IK（有欧拉则含姿态）
2. 有 `MotionAxisConfiguration` 约束 → `solveIkWithAxisConfiguration`（无解且显式约束 → **失败，不回退**）
3. DH `ikPositionDampedLeastSquares`（`hasDhRows()`）
4. URDF 重试 / legacy 单关节增量

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
| `tryStart(doc, osg, io, robotInstanceIndex, program, motionPlanResults, initialAngles, err)` | 含 IF/WHILE/WAIT/IO |
| `tick(doc, osg)` | 状态机 + 运动段 |
| `stop()` / `isRunning()` | 控制 |

私有：`While` 最大迭代 `kMaxWhileIterations = 10000`。

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
| `fkMeshWorldT0` | bind 时各 link 网格世界矩阵 |
| `outerWorldAtBindByBackendId` | bind 时 outer 世界矩阵 |
| `meshVerticesInLinkFrame` | 顶点是否在连杆系 |

### 8.2 `namespace RobotSceneKinematics`

| 函数 | 作用 |
|------|------|
| `applyJointAnglesFromDocument(doc, osg, anglesRad)` | 按实例循环 per-link / 层级 |
| `applyJointAnglesForInstance(doc, osg, instanceIndex, local, aggregated)` | 单机 |
| `applyJointAnglesViaLinkBackends(doc, osg, mgr, angles, slice)` | `Mnew = M0 * inv(T0) * Tq` 写 outer + `setWorldMatrix` |
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
| `RobotUserFrame` | `id` / `name` / `T_base_user` |
| `RobotCoordinateFrameSet` | `toolFrames[]`（`T_flange_tool`）、`activeToolFrameId`、`flangeLinkName`、用户系列表、激活 id、3D 显示开关 |

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

**指令扩展键（PTP/LINE）**

| 键 | 说明 |
|----|------|
| `motion.tool.frameId` | 工具系 id；切换时 **保持 TCP 空间位置**（`pose` 不变），重算 IK |
| `motion.user.frameId` | 用户系 id |
| `motion.target.frame` | `base` / `user` — 面板显示系 |
| `context.toolFrameMat4` | 规划用工具矩阵（按点冻结） |
| `context.poseFrame` | `base_tool_origin`（兼容 `base_tcp`） |
| `context.flangeLinkName` / `context.tcpLinkName` | 法兰 link（IK/FK） |

`[IK残差]` 分列：`toolOrigin`、`flangeTarget`、`fkFlange`、`fkToolOrigin`；主指标 **`residualTcpMm`** = \|toolOrigin − fkToolOrigin\|（FK 经 `toolOriginFromFlange`）。

规划上下文：按点写入工具矩阵与法兰 link。指令 `pose` = **基座下工具原点**；3D 轴与绿/红可达性同前。

### 工具链 FK（禁止误用 OSG 裸乘）

1. `T_base_flange`：`UrdfRobotLoader::computeLinkWorldMatrices` → `engine::rigidTransformFromOsg(linkWorld[flange])`。
2. `T_flange_tool`：`rigidTransformFromFrame` 或 `rigidTransformFromBackendMat4(context.toolFrameMat4)`。
3. `T_base_target = engine::toolOriginFromFlange(T_base_flange, T_flange_tool)` — **必须**走 `ToolKinematics`（内部 `composeColumn`）。
4. IK 前置：`flangeFromToolOrigin(T_base_target, T_flange_tool)`（`ikLinkTargetFromInstruction`）。

**反例（已修复）**：`linkWorld * osgMatrixFromBackendMat4(T_tool)` 或 `composeScene` 组合 URDF 法兰与 Eigen 工具 → 法兰系 `(0,0,-200)` 可能表现为仅基座 Z 变化。`RobotMatrixOsg::targetInBaseFromFlangeLinkWorld` 与 `Widget::osgTcpInBaseFromFlangeLinkWorld` 已改为 engine 路径。

**验收**：法兰 Ry≈90° 时，工具 Z=-200 应在基座 X（或 Y）体现约 200 mm 偏移，而非仅 ΔZ。见 `GeometryEngine::runSelfTest`。

## 8.4 程序导出（`RobotProgramExport.h`）

| API | 作用 |
|-----|------|
| `buildExportResult` | 运动点列表 + 规划结果 → 基座 TCP + 关节角 |
| `writeExportResultToJson` / `writeExportResultToCsv` | 导出（`coordinateFrame: base_tool_origin_mm_deg`，语义同示教 `pose`） |

仿真 Dock **Export…** 先链式 `plan` 再写文件；失败点 `ikOk=false` 并带 `ikError`/`summary`。

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

**预览**（非运行）：`RobotSimulationController::applyRobotPoseForInstructionPreview` 自 `m_motionPreviewProgramStartJointRad` 链式至选中点。若该点含 `context.currentJointRadCsv`（添加指令时写入），**直接**用示教关节角，不对该点重算 IK；否则 `validate` + `plan`。轴配置可行列表 `queryFeasibleMotionAxisConfigurationOptions` 带缓存。

**运行**：`onSimulationStartTriggered` 对每条运动同样优先示教 CSV 构建 `PlanResult::jointTargetsRad`；`RobotProgramExecutor` 插值执行。程序起点仅在**第一条**运动指令加入时更新（见 [`../RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md)）。

**末端拖动示教**（非运行、不写指令）：屏幕空间平移更新 `T_base_target` → `RobotTeachIk` → 关节钳位 → `applyJointAnglesForInstance`；添加指令时用罗盘位姿 + `currentJointRadCsv` 落盘。见 [`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) §13.1、[`../RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md)。

### 11.1 `RobotTeachIk`

| API | 说明 |
|-----|------|
| `TeachIkContext` | `urdfPath`、`ikLinkName`、`T_base_target`、`seedJointRad`、`T_flange_tool` |
| `solveTeachIk` | 交互示教 IK；法兰目标经 `engine::flangeFromToolOrigin` |

---

## 12. 相关文档

- 刚体/工具链：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md) · [`../GeometryEngine/CONVENTIONS.md`](../GeometryEngine/CONVENTIONS.md)
- URDF：[`../RobotUrdf/DEVELOPER_GUIDE.md`](../RobotUrdf/DEVELOPER_GUIDE.md)
- DH：[`../RobotKinematics/DEVELOPER_GUIDE.md`](../RobotKinematics/DEVELOPER_GUIDE.md)
- UI：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) §仿真
- 轴配置详解：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §4.8.1–4.8.2
