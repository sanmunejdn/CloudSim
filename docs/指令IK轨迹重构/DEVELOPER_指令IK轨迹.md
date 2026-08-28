# 指令 / IK / 轨迹四层

1. **Instruction（持久化）**：仅 TCP（pose/euler + 变换扩展）+ 运动参数 + 轴配置；禁止关节 CSV。
2. **PlanRequest / IkRequest**：临时；`IkSeedPolicy` = `FromCurrentPose` | `FromInstruction`；种子由 UI 解析后注入 `prepareInstructionIkContext`（backup/restore）。
3. **IK（KinematicCore）**：Pinocchio 形契约（目标 SE3 + q 种子）；本阶段不整库 Pinocchio。
4. **Ruckig（PTP）**：`PtpPlanner` 在 IK 成功后生成 `jointTrajectoryRad`；回放引擎按轨迹插补。LINE/ARC 仍笛卡尔采样。

详见 [CONSENSUS_指令IK轨迹重构.md](./CONSENSUS_指令IK轨迹重构.md)。
