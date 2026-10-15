# CONSENSUS — 仿真统一内核

## 1. 锁定决策

| 项 | 约定 |
|----|------|
| 范围 | 全量 P0–P5 |
| IO 可见性 | **下一拍**：本拍 Wait/IF 读上一拍 Sample；本拍 commit+propagate 写入 next-sample |
| 租约冲突 | **立即 Abort** 冲突 Lane；其它 Lane 继续 |
| StopAll | 仅 CollisionGuard 策略使用，不用于租约 |
| 设备指令子集 | GoNamedPose、DeviceAxis、SET_DO/AO、WaitSignal、IF/WHILE、SyncBarrier、Acquire/Release |
| 协调运动 | P4 SyncBarrier；P5 不做多臂相对轨迹，只做 Authority + CollisionGuard |
| 外置 | 按实例 ControlAuthority；pendingTargets 交 Kernel 统一 Apply |

## 2. 验收标准（总）

1. 单机 Run 行为与改造前等价（允许握手晚 1 tick）
2. Run All：多机并行，关节互不覆盖
3. 两机抢同一设备资源 → 冲突方 Abort，可诊断
4. 机 DO → wire → 设备程序/姿态 → Done → 机 Wait → 继续
5. 挂载设备统一 Run 无闪裂
6. SyncBarrier 后多 Lane 对齐进入下一段
7. 外置占用实例不被程序 Lane 双写
8. 碰撞策略 FreezeLane / StopAll 可复现
9. RobotScene + RobotWidget（及必要时 Host）Debug\|x64 与 Release\|x64 编译通过

## 3. 技术约束

- Kernel 落在 **RobotScene**（无 Qt Widget 依赖）
- 禁止 DevicePoseMotionPlayer 独立 QTimer 写场景
- 写场景出口：WorldApplyOrder → KinematicModelRegistry / CustomDeviceKinematics
- 桌面/Host IO 共 `IIoSignalNetwork` 语义（下一拍）

## 4. 任务边界

见 `TASK_仿真统一内核.md` 阶段划分；品牌导出、硬实时、力控不在本共识内。
