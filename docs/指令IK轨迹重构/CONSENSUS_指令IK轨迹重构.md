# CONSENSUS_指令IK轨迹重构

## 需求描述

指令对象只持久化最终 TCP 位姿；关节求解全部交给求解器。IK 种子可选「当前位姿」或「指定指令点」。PTP 关节段用 Ruckig 生成限速/加/jerk 轨迹。Pinocchio 仅对齐 API 形，本阶段不整库接入。

## 指令持久化白名单

- TCP：`pose` / `eulerDeg`，及 `context.targetTransformQuatCsv` + `context.targetTransformTransMmCsv`（有 euler 时读以 pose+euler 为真源）
- ARC via：viaPose / viaEuler 及对应 via 扩展
- 运动参数：speed / accel / blendRadius
- `MotionAxisConfiguration`
- 工具引用：`motion.tool.frameId` 等坐标系引用（非关节缓存）

## 禁止持久化键

- `context.currentJointRadCsv`（仅规划调用期间可作为临时种子注入，结束后须还原/不落盘）
- `context.taughtJointRadCsv` / DTO `taughtJointRadCsv`
- 任意「免 IK」关节短路缓存

## 契约

### PlanRequest（临时，不落盘）

- `targetTcp`：来自指令
- `seedPolicy`：`FromCurrentPose` | `FromInstruction`
- `seedInstructionId`：FromInstruction 时
- `qSeedResolved`：解析后的种子关节
- urdf / tool / limits

### IkRequest（Pinocchio 形）

- `targetTcpInBase` + `qSeedResolved` + solver options
- 后端仍为 KinematicCore DLS

### PlanResult

- `ok` / `jointTargetsRad` / `jointTrajectoryRad` / `durationSec` / 外轴
- 不写回指令关节字段

## Seed 解析

1. `FromCurrentPose`：轴控/聚合当前关节（按 instance 偏移切片）
2. 链式 Run：上一句 `PlanResult.q`（空 `seedInstructionId` 时等价自动 FromInstruction(prev)）
3. `FromInstruction(ref)`：会话有 Ref 的成功 `PlanResult` 则用之；**无缓存则硬失败**（禁止静默降级到当前位姿）。先解 Ref TCP 属后续增强，本阶段不做。

## 防卡死（保留）

- UI 单层随机重启，预算 400ms
- `maxIterations=80`，`maxPosThenOriAttempts=6`
- IK 核内无随机重启

## 验收标准

1. Run 每点必走 IK，无 taughtJointCsv 短路
2. 示教/路径规划成功只写 TCP
3. 单点可选两种种子策略
4. PTP 段 Ruckig 轨迹回放；LINE/ARC 仍笛卡尔采样 + IK
5. Debug|x64 与 Release|x64 相关工程编译通过
