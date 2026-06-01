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
| `EntireProgram` | `collectMotionInstructions` 全部 PTP/LINE |
| `Group` | 匹配 `scope.groupId` 的 `memberInstructionIds`（**不**过滤运动类型；若分组不存在则返回空列表，由 UI `reconcilePipelineScopes` 协调） |
| `PointIndexRange` | `motionPointIndex` ∈ [pointFrom, pointTo]（扩展键 `motion.pointIndex`） |
| `InstructionIds` | 显式 id 列表 |

---

## 13. 轨迹流水线与程序编辑 Command

### 13.0 算法库与桥接

| 工程 | 说明 |
|------|------|
| [`TrajectoryAlgorithm`](../TrajectoryAlgorithm/DEVELOPER_GUIDE.md) | `ITrajectoryOp`、Registry、ParamSchema、Codec、`TrajectoryTransformMath` |
| [`TrajectoryAlgorithmBuiltins`](../TrajectoryAlgorithmBuiltins/) | Translate / Rotate / Delete / Duplicate / Mirror(轴反向) / Reorder(固定姿态) / RecipeWeld / RecipeGlue / RecipeGrind / Approach / Retract |
| [`TrajectoryOpBridge.h`](inc/TrajectoryOpBridge.h) | **UI 唯一入口**：Registry、参数读写、模板 JSON（避免 RobotWidget 重复链接静态库） |

`ensureTrajectoryOpBuiltinsRegistered()` 在 `TrajectoryPipelineBuilder::buildApplyCommands` 与 UI 构造时调用。Apply：`ITrajectoryOp::buildApplyActions` → [`TrajectoryApplyActionConverter`](source/TrajectoryApplyActionConverter.cpp) → `ProgramEditCommand`。

### 13.1 流水线描述符

| 类型 | 说明 |
|------|------|
| `TrajectoryOpKind` | Translate / Rotate / Mirror(轴反向) / Delete / Duplicate / Reorder(固定姿态) / RecipeWeld / RecipeGlue / RecipeGrind / Approach / Retract |
| `TrajectoryOpDescriptor` | `kind` + `OpScope` + `translate` / `rotate` / `duplicateCount` 等 |
| `TransformReferenceFrame` | `World` / `Body`（`TranslateParams` / `RotateParams` 的 `frame`） |
| `TrajectoryPipelineBuilder` | `setOps` + `buildPreviewPoseQuery` + `buildApplyCommands`（内部走 Registry） |

预览链：`buildPreviewPoseQueryChain` 按流水线顺序调用各块 `contributePreviewTransform`（`capabilities & PreviewPoseTransform`），叠加到 `TransformMotionPoseQuery`；位姿合成用 `trajectory_algo::applyTransformDelta`。

### 13.2 `ProgramEditCommand` / `ProgramEditStack`

| Command（示例） | 作用 |
|-----------------|------|
| `TransformMotionSegmentCommand` | 平移/旋转 scope 内路点 |
| `InsertInstructionCommand` / `RemoveInstructionCommand` | 增删（Duplicate 等） |
| `CreateInstructionGroupCommand` | 创建分组 |
| `RemoveInstructionGroupCommand` | 解散分组（不删指令） |
| `RenameInstructionGroupCommand` | 重命名分组 |

`InstructionProgramDocument`：在 `activeProgram()` 步骤树上按 id 查找/修改。Command 使用 `shared_ptr` 跨 DLL 边界（`ProgramEditStack::CommandPtr`）。

Apply 路径分两支：Program Command 分支（`TrajectoryPipelineBuilder::buildApplyCommands` → `ProgramEditService::executeBatch`）与 Unified IR 分支。2026-05 修复后，**RawTrajectory 存在时一律优先走 Unified IR**（不再仅限 Recipe/Approach/Retract），避免“生成前平移/旋转无效”。Translate/Rotate 在两条分支都按点序支持起点→终点线性插值。

UI 侧新增门控：Apply 成功后会禁用“生成程序”入口，并在 `onRawEmitProgram` 做硬门禁，避免 `emitRawTrajectoryToProgram` 清空主程序后覆盖 Apply 结果。

撤销/重做后流水线 scope 与程序分组可能不一致（例如撤销「创建分组」）；UI 层 `TrajectoryEditPageWidget::reconcilePipelineScopes` 在 Preview/Apply 与 `revisionChanged` 时回退失效的 `Group` scope，详见 RobotWidget §轨迹编辑。

业务编排与 UI 绑定见 [`../RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md) §轨迹编辑。

---

## 14. CAD 轨迹中间表示（`RawTrajectory.h`）

与 §13 **程序级**轨迹编辑并行，特征→轨迹仍基于 `RawTrajectory`，并新增 UnifiedTrajectory 承接 Recipe 与通用块：

```text
FeatureSpec → discretizeFeature → RawPath → importRawPathToTrajectory → RawTrajectory
  → Recipe/Approach/Retract/通用块（UnifiedTrajectory）→ ReplaceProgramContentCommand → RobotProgram
```

| 类型 | 说明 |
|------|------|
| `RawTrajectory` | `points` + `TrajectoryContext` + 溯源 `sourceFeature` |
| `TrajectoryPoint` | `poseMm`、`eulerDeg`、`blendRadiusMm`、`speedMmPerSec`、`reachable` |
| `TrajectoryContext` | `workpieceFrameId`、`toolFrameId`、`externalAxes[]`（地轨/变位机快照，非工艺字段） |
| `RawTrajectoryOpKind` | `FrameFromPath`、`Resample`、`OffsetAlongNormal`、`OffsetLateral`、`SmoothPose`、`AssignBlend`、`AssignSpeedZone`、`Weave`、`InsertApproachRetract`、`ReachabilityFilter`、`ExternalAxisSearch`、`EmitToProgram` |
| `RawTrajectoryOpDescriptor` | 单块参数（步距、偏置、摆焊振幅/周期等） |
| `UnifiedTrajectory` | 统一点位表示（pose/euler/blend/speed/reachable），用于 Recipe 与通用块的统一执行 |

| API | 作用 |
|-----|------|
| `importRawPathToTrajectory` | `RawPath` + `FrameStrategy`（法向 Z / 固定 Z / 切向 X）→ 姿态 |
| `applyRawTrajectoryOp` / `applyRawTrajectoryPipeline` | 编辑块；`EmitToProgram` 须用 `emitRawTrajectoryToProgram` |
| `rawTrajectoryRecipeWeldDefault` / `Glue` / `Grind` | 配方底座（RecipeBlueprint 会按新策略剥离内置进退刀） |
| `buildRecipePreset` / `applyRecipeDescriptorToRawTrajectory` | 配方模板与 raw 配方映射 |
| `unifiedTrajectoryToRaw` | 生成前预览回写：Unified 结果映射回 Raw overlay |
| `emitRawTrajectoryToProgram` | 可达点 → `LineInstruction` 序列写入 `RobotProgram.steps`，并默认创建 `InstructionGroup`（组名 = `featureId`） |
| `rawTrajectoryToPreviewPolylineXyz` / `rawTrajectoryReachabilityColorsJson` | UI/OSG 预览 |

实现：[`source/RawTrajectory.cpp`](source/RawTrajectory.cpp)。

**与 `TrajectoryAlgorithm` 的关系**：`ITrajectoryOp` 是统一块描述入口；Recipe/Approach/Retract 在 Session 的 Unified IR 分支执行，Translate/Rotate/Mirror/Reorder 同时支持 Program 分支与 Unified IR 分支。

**Phase 占位**：`ReachabilityFilter` 当前为轻量启发式标记；完整 IK 可达性可接入 `RobotInstructionController::plan` / `queryFeasibleMotionAxisConfigurationOptions`。

---

## 15. 相关文档

- 轨迹算法：[`../TrajectoryAlgorithm/DEVELOPER_GUIDE.md`](../TrajectoryAlgorithm/DEVELOPER_GUIDE.md)
- 刚体/工具链：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md) · [`../GeometryEngine/CONVENTIONS.md`](../GeometryEngine/CONVENTIONS.md)
- URDF：[`../RobotUrdf/DEVELOPER_GUIDE.md`](../RobotUrdf/DEVELOPER_GUIDE.md)
- DH：[`../RobotKinematics/DEVELOPER_GUIDE.md`](../RobotKinematics/DEVELOPER_GUIDE.md)
- 特征离散：[`../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md`](../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.1
- UI 轨迹生成：[`../RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md) §CAD 轨迹生成
- 轴配置详解：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §4.8.1–4.8.3
