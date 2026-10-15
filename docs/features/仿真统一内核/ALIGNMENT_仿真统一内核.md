# ALIGNMENT — 仿真统一内核

## 1. 项目上下文

| 项 | 说明 |
|----|------|
| 编排现状 | `RobotSimulationController` 持有单个 `RobotProgramExecutor`，Run 仅 `currentRobotInstanceIndex` |
| 设备现状 | `DevicePoseMotionPlayer` 自有 QTimer；`DevicePoseSignalDriver` DI 上升沿旁路驱动 |
| IO | 桌面 `IoSignalNetworkService`；Host `IoSignalNetwork`；连接站 DO→DI |
| 运动学 | 多实例 `applyJointAnglesForInstance`、`KinematicModelRegistry`、挂载 Follow 已存在 |
| 程序存储 | 每机器人 `RobotProgramCatalog`；设备仅姿态库 + `poseSignalBindings` |

## 2. 原始需求

- 多台机器人程序同时运行
- 自定义设备与机器人在统一运行会话下信号交互
- 用架构消除：挂载顺序、多写者竞态、IO 时序不确定、外置双写、缺少同步屏障与运行时碰撞联锁

## 3. 边界

**在范围内**

- RobotScene 内 `SimulationKernel`：Lane / 逻辑时钟 / 租约 / IO 相位 / WorldApplyOrder / SyncBarrier / ControlAuthority / CollisionGuard
- UI 变薄门面；DevicePlayer 去掉独立定时器
- 设备程序侧车与指令子集
- Host/桌面 IO 契约对齐（下一拍采样）

**不在范围内**

- 硬实时 / 真 PLC 同步
- 多臂相对轨迹规划 / 力控
- 品牌程序多机合并导出

## 4. 需求理解

缺的是「多会话编排 + 写权 + IO 相位」，不是再做一个执行器塞多机队列。设备应升为 DeviceLane，与机器人共用 Kernel 时钟与租约。

## 5. 疑问澄清（已锁定）

见 `CONSENSUS_仿真统一内核.md`。
