# CONSENSUS_机器人挂载跟随修复

## 需求描述

修复「自定义设备挂载机器人法兰后，机器人程序播放（Run）期间设备不跟随、运动结束后瞬移」的缺陷。从框架层面消除「机器人当前关节角」状态真源分裂：凡把关节角应用到场景的路径，必须在同一抽象层回写关节角真源；读取方（挂载设备 TCP 重算）只依赖真源，不依赖调用方约定。

## 技术方案（方案一：doc 级关节角真源，FK 收口写入）

1. `IRobotSimulationDocument` 新增虚函数 `noteRobotJointAnglesAppliedForInstance(int instanceIndex, const QVector<double>& localJointRad)`，默认空实现；
2. 两个 FK 收口点回写（RobotScene 层）：
   - `applyJointAnglesFromDocument` per-instance 循环：per-link 分支与层级矩阵分支各回写一次；
   - `UrdfRobotKinematicModelSink::applyToSink` per-link 分支（`applyLinkWorldFromCoreFk` 成功后、notify 前）；
3. 实现侧：
   - `DocumentPage`（IS-A `DocumentHost`）：`robotSceneBackendIdForInstance(instIdx)` 解析 sceneRoot → 写 `DocumentHost` 缓存；
   - `MainWindowRobotHost::DocumentHost` 包装类：转发 `m_page`；
   - `HeadlessRobotContext`：写自有缓存（`recordJointAnglesForSceneRoot`）+ host 缓存（播放桥种子读）；
4. `DocumentHost::noteRobotLocalJointAnglesForSceneRoot` 改为**直写 local** 语义（去除聚合切片，原调用方全部清除）；
5. 事件纯化：`publishRobotKinematicsApplied` 仅发 `RobotKinematicsAppliedEvent`，不再写缓存；`CustomDeviceHostOps.cpp` 挂载前 FK 后的两处显式回写删除（已被收口覆盖）。

## 技术约束与集成方案

- 遵守分层：RobotScene 不反向依赖 CloudSimHost/Widget，回写经 `IRobotSimulationDocument` 虚函数下沉到实现侧；
- vtable 变更：`IRobotSimulationDocument` 全部实现（`DocumentPage`、`HeadlessRobotContext`、`MainWindowRobotHost::DocumentHost` 经 `IRobotDocumentHost` 继承）同步重编；使用该接口的 DLL（RobotScene/CloudSimHost/Widget/RobotWidget/CloudSimWebGateway）需一致重建；
- 回写不经过 `KinematicsBatchScope` 的 defer：apply 当场写缓存，batch 末 follow 刷新读到的即最新值，时序正确；
- 注释规范：纯中文、聚焦 Why，不复述代码行为。

## 任务边界限制

- 不改 `resolveEffectiveDeviceW0`、不改 `DevicePoseMotionPlayer` 定时器、不改 Follow 求解器跳过语义；
- 不改 `WebGatewayApi.cpp` 的显式 hrc 回写（幂等无害）；
- 不改任何 OutDir/输出路径；产物以各 vcxproj 为准。

## 验收标准

1. 挂载设备 + 机器人 Run：设备全程逐帧跟随法兰，不再钉在起点；
2. Run 期间叠加设备姿态运动（运动到此 / DI 触发）：两者平滑叠加；
3. 含 DeviceAxis 段与运动段交替的程序正常；
4. 停止后设备位置 == 运行末帧位置（无瞬移）；
5. 回归：手动轴控跟随、TCP 拖动示教、点击预览、工程保存/加载后挂载位姿、Headless 挂载刷新；
6. 编译：RobotScene → CloudSimHost → Widget → RobotWidget（及 CloudSimWebGateway）依赖链，Debug|x64 与 Release|x64 各编一遍通过。

## 不确定性确认

全部关键假设已在 ALIGNMENT 中确认（包装类转发必要性、Headless 共用 Executor、桌面 doc 直达）；无遗留歧义。
