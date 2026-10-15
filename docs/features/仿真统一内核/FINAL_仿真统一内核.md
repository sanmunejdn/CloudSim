# FINAL — 仿真统一内核

## 交付摘要

已在 RobotScene 落地 `SimulationKernel`（Lane / LogicalClock / IoPhase 下一拍 / ResourceLease Abort / WorldApplyOrder / SyncBarrier / ControlAuthority / CollisionGuard），RobotWidget 接入 Run / Run All、设备 Player 去独立 QTimer，Host 提供 `HostIoSignalNetworkAdapter`。

跟踪单：**#48**

## 主要产物

| 区域 | 文件 |
|------|------|
| Kernel | `RobotScene/inc/SimulationKernel.h` 及 IoPhase/Lease/Lane/Barrier/Authority/CollisionGuard |
| 指令 | `GoNamedPose` / `SyncBarrier` / `AcquireResource` / `ReleaseResource` |
| 设备程序 | `DeviceProgramStore` |
| UI | `RobotSimulationController` → Kernel；`Run All`；`DeviceMotionTickerAdapter` |
| Host | `HostIoSignalNetworkAdapter.h` |
| 文档 | `docs/features/仿真统一内核/*` |

## 编译

| 工程 | Debug\|x64 | Release\|x64 |
|------|------------|--------------|
| RobotScene | 通过 | 通过 |
| RobotWidget | 通过 | （本报告撰写时同步编） |

## 已知限制

- Run All 逐台复用单机规划路径，重程序时启动较慢
- Host 多 Owner 细粒度 sink 仍简化为网自身 IRobotIoSink
- CollisionGuard 需 UI 注入 probe 后才生效
- 挂载 DAG 边需业务填充 `WorldApplyOrder::setMountEdges`
