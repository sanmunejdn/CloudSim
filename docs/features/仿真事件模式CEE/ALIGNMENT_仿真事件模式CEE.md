# ALIGNMENT — 仿真事件模式（薄 CEE）

## 1. 项目上下文

| 项 | 说明 |
|----|------|
| 编排 | `SimulationKernel`：Lane / LogicalClock / IoPhase 下一拍 |
| IO | 桌面 `IoSignalNetworkService`；DI→姿态 `DevicePoseSignalDriver` |
| 缺口 | 无 Study 级「基于时间 / 基于事件」切换；无进程内事件求值器 |

## 2. 原始需求

阶段 A：薄 CEE MVP——模式切换 + 规则表启停资源；EventBased 下 CEE 唯一逻辑权威。

## 3. 边界

**在范围内**

- `TimeBased` \| `EventBased`
- `CeeEvaluator`（risingEdge / levelHigh / levelLow → goNamedPose / startRobotProgram / setDo / stopLane）
- Kernel tick 在 `swapAll` 后 evaluate
- EventBased 禁用 `DevicePoseSignalDriver`；可从 `poseSignalBindings` 同步规则
- `devicePrograms` / `simulationRunMode` / `ceeRules` 工程存取（桌面）

**不在范围内**

- 外接 OpenPLC / 实 PLC 作逻辑权威
- Appearance、物料流、刚体、ProcessFlow 合并
- Host / 网页事件模式 UI
- IEC / 梯形图编辑器

## 4. 需求理解

缺的是同一 3D Study 上的事件 Runner 与模式门面，不是再做一套 DES。
