# ALIGNMENT_机器人挂载跟随修复

## 原始需求

机器人运动与自定义设备运动单独执行均正常；但自定义设备挂载到机器人法兰后两者同时动作时，设备**不跟随**机器人运动（自身轴运动正常），并在机器人运动结束后**瞬移**到正确位置。要求从框架层面彻底修复。

## 需求理解（对现有项目的分析结论）

### 根因

「机器人当前关节角」状态真源分裂：

- 连杆世界矩阵（Data/OSG）：**所有** FK 路径都会写；
- 关节角缓存 `DocumentHost::m_robotLocalJointQBySceneRoot`：**仅** `RobotServiceAdapter::applyJointAnglesRad`（手动轴控/预览/TCP 拖动/停止落姿）经 `publishRobotKinematicsApplied`（`DocumentHostEvents.cpp:45`）写入。

挂载设备世界位姿的唯一真源是 `refreshCustomDevicesFollowingKinematicsTargets` → `updateMountedDeviceWorldFromRobotTcp`（Follow 求解器对已挂载设备 `isPoseExternallyDriven()` 跳过，设计使然），其 TCP 重算经 `tryResolveMountFlangeTcpFromUrdfFk` 读取上述关节角缓存（`CustomDeviceRobotMountOps.cpp:263`）。

播放路径（`RobotProgramExecutor::tick` → `applyJointAnglesFromDocument`）每帧确实触发 follow + 挂载刷新（`DocumentPage::notifyRobotKinematicsAppliedToScene`，`DocumentPage.cpp:1187`），但刷新用的是**陈旧缓存**，每帧算出同一个旧 TCP → 设备钉在原地；`stopRobotSimulation` 末尾 `applyJointAnglesRad`（`RobotSimulationController.cpp:1284`）刷新缓存 → 瞬移。

### 两条平行 FK 管线

| 路径 | 入口 | 收口 | 写关节缓存 |
|---|---|---|---|
| 手动（轴控/预览/TCP/停止） | `RobotServiceAdapter::applyJointAnglesRad` | `KinematicModelApply::applyRobotArm` → `UrdfRobotKinematicModelSink::applyToSink` → notify | 是（adapter 尾部） |
| 播放（Run 程序） | `RobotProgramExecutor::tick` | `RobotSceneKinematics::applyJointAnglesFromDocument` → notify | **否** |

### 补充发现

- `CustomDeviceKinematics.h:24` 注释声明「挂载启用时 W_eff = T_flange_world * T_flange_device」，实现（`CustomDeviceKinematics.cpp:117-124`）未落实该分支（仅返回 `device.worldMatrix()`）。经评估不作为本方案兜底（法兰 Data 矩阵在装配系网格下含 visual 偏移，与 URDF 法兰系不等价）。
- Headless 侧存在双缓存：`HeadlessRobotContext` 自有缓存（挂载 TCP 重算读）+ `DocumentHost` 缓存（`HeadlessRobotPlaybackBridge` 起点种子读）。修复需两端一致。

## 边界确认

### 范围内

- `IRobotSimulationDocument` 新增关节角回写虚函数（默认空实现）；
- FK 两个收口点回写：`applyJointAnglesFromDocument`（播放/预览/工程加载/URDF 导入/非 per-link 回退）、`UrdfRobotKinematicModelSink::applyToSink` per-link 分支（手动 adapter 路径；Composite 模型汇入同一 sink）；
- 实现侧：`DocumentPage`、`MainWindowRobotHost::DocumentHost` 包装转发、`HeadlessRobotContext`（双缓存）；
- `DocumentHost::noteRobotLocalJointAnglesForSceneRoot` 改为直写 local 语义（原聚合切片版调用方全部在本任务内清除）；
- 事件纯化：`publishRobotKinematicsApplied` 移除缓存写入，仅发 `RobotKinematicsAppliedEvent`；清理 `CustomDeviceHostOps.cpp:136-137` 冗余回写；
- Debug|x64 与 Release|x64 双配置编译验证。

### 范围外（不做）

- `resolveEffectiveDeviceW0` 挂载分支（方案二，已否决：visual 偏移歧义）；
- 统一运动时钟/场景运动调度器（方案三，列入 TODO 演进方向）；
- `DevicePoseMotionPlayer` 独立 16ms 定时器、Follow 求解器 `isPoseExternallyDriven` 跳过语义；
- `WebGatewayApi.cpp:545-549` 的显式 `recordJointAnglesForSceneRoot`（幂等同值写入，无害，保留）。

## 疑问澄清（已决策）

| 决策点 | 结论 |
|---|---|
| 修复范围 | 仅方案一（统一关节角真源） |
| 接口 ABI | 接受 `IRobotSimulationDocument` 加虚函数，所有实现同步重编 |
| 事件纯化 | 缓存单一写源在 FK 收口；adapter 只发事件 |

## 关键假设

- `MainWindowRobotHost::DocumentHost` 包装类必须转发新虚函数，否则播放路径（`m_host->document()` 返回包装）命中基类空实现，bug 依旧；
- Headless 播放与桌面共用 `RobotProgramExecutor`，同一收口点覆盖；
- 桌面 `urdfImportRobotSimulationDocument()` 返回 `DocumentPage` 自身（`DocumentPage.h:152`），手动路径收口写入直达实现。
