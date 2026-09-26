# 指令 / IK / 轨迹四层

1. **Instruction（持久化）**：仅 TCP（pose/euler + 变换扩展）+ 运动参数 + 轴配置；禁止关节 CSV。存储位姿恒为 **基座（P0）工具原点**；User 系仅显示层。
2. **PlanRequest / IkRequest**：临时；`IkSeedPolicy` = `FromCurrentPose` | `FromInstruction`；种子由 UI 解析后注入 `prepareInstructionIkContext`（backup/restore）。会话级 `context.allowApproximateOrientation`（默认 false）控制 SoftAccepted。
3. **IK**：`solveArmPoseDampedLeastSquares` — 球形腕 6R 解析优先，否则 KinematicCore DLS；有 URDF/TCP 时禁止 DH/legacy 位姿回退。无 URDF 且无姿态时可 DH/legacy（summary 标 `ikPath=`）。显式轴配置：腕连续门硬失败；外轴联立成功须过残差门限。
4. **Ruckig（PTP）**：`PtpPlanner` 在 IK 成功后生成 `jointTrajectoryRad`；回放引擎按轨迹插补。LINE/ARC 仍笛卡尔采样。

详见 `docs/ARCHIVE_ZIP_LOCATION.txt`。
