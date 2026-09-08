# ACCEPTANCE_指令IK轨迹重构

## 已完成

- [x] CONSENSUS / DEVELOPER 文档
- [x] 指令 TCP-only：禁用 taught 短路；示教/Raw 不落盘关节 CSV
- [x] `IkSeedPolicy` + `IkRequest`；`setIkSeedPolicy` / `setIkSeedInstructionId`
- [x] Vendor Ruckig；`PtpPlanner` 生成 `jointTrajectoryRad`
- [x] PlanResultCache 保留稠密轨迹；回放不 prepend
- [x] FromInstruction 无缓存硬失败；Current 按 instance 切片
- [x] R1 折圈同步平移轨迹；剥轨迹后复验碰撞
- [x] Host 序列化忽略关节 CSV；PTP 限速统一 `ruckigLimitsFromPtpDegScalars`
- [x] Lookahead 入库前 R1+align；剥轨迹复验碰撞
- [x] PlanResultDto / Web 回传 `jointTrajectoryRad` + `durationSec`
- [x] `toJson`/`createFromJson` 剥离禁止落盘关节键
- [x] 仿真页 IK种子下拉（链式 / 当前）接线
- [x] Debug|x64 + Release|x64：RobotScene、RobotWidget、CloudSimHost 编译通过

## 手工回归建议

1. 开工程 Run：应走 IK，无卡死、无 taughtJointCsv 短路
2. PTP 段缓存命中后仍见 Ruckig 型线（非纯端点 lerp）
3. 仿真页选「当前」后单点预览：种子应来自轴控当前关节
4. 改路点 TCP 后重 Run：旧 PlanResult 不复用
5. Host/Web `planInstruction` 响应含 `jointTrajectoryRad` / `durationSec`
6. 再存工程：`context.currentJointRadCsv` 不应再写出
