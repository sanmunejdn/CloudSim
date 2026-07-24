# CONSENSUS — 外部轴类型拓宽

## 需求与验收
1. 可配置 ≥2 轴，任意 Translate/Rotate × RobotBase/Workpiece，存盘重开
2. 轴控拖动时机器人或绑定工件姿态连续正确
3. 多轴时 PTP/LINE/示教/Run 写出并播放完整 `externalAxisQs`
4. 单轴旧工程回归
5. `RobotScene` + `RobotWidget` Debug|x64 编译通过

## 技术方案
- 配置：`motionType` / `attachment` / `axis` / `originMm` / `boundBackendId` / 可选 `workingFrameId`
- FK：`P_eff = P0 * ΠT_i`；`W_eff = W0 * ΠT_i` → `setBackendRootWorld`
- IK：外轴稀疏采样 + 臂 URDF IK；DOF≤2 平移可联立 DLS
- 指令：`context.externalAxisQCsv` + 兼容 `externalAxisQMm`
- PlanResult：`externalAxisQs`（config 下标对齐）+ 标量兼容

## 二期（Workpiece REP，已锁定）
- 启用 Workpiece 时示教/规划 TCP 按相对工作架 `T_work`；采样 `q_w` 重建 `T_p0_goal = T_p0_work(q_w)*T_work`，内层仅 RobotBase TeachIk
- 扩展键：`context.workingTcpTransMmCsv` / `context.workingTcpQuatCsv`
- 文档：`workpieceWorkingFrameOffsetByBackend`；Host 注入 `WorkpieceIkFrameContext`（含异步 PlanJobPayload）
- 明确不做：品牌 E1/E2、闭链、Tesseract/MoveIt 依赖

## 约束
不引入 ROS 运动学栈；不改闭链语义。
