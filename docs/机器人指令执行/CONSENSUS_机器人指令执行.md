# CONSENSUS：机器人指令执行 / 预览 / 运行

## 需求与验收

| 项 | 约定 |
|----|------|
| 运算模型 | Plan 批算 + Play 时间插值；非每帧在线 IK |
| 预览 | 选中点 1× IK（或示教 CSV），一帧到位 |
| Run 部分失败 | 播到第一个失败运动点前停止；成功前缀可播；不跳过失败点 |
| 全失败 | 不启动仿真 |
| LINE | URDF 可用时笛卡尔采样 + 逐点 IK；否则关节空间回退 |
| 播放轨迹 | `jointTrajectoryRad.size()>=2` 优先，否则 `jointTargetsRad` 起止 lerp |

## 技术约束

- 编排：`RobotSimulationController::onSimulationStartTriggered`
- 执行：`RobotProgramExecutor`（`Aborted` + `abortedDueToFailedPlan`）
- 规划：`PtpPlanner` / `LinePlanner`（`RobotInstructionController.cpp`）
- 失败结果不写入 `PlanResultCache` 成功路径；失败用 `ok=false` 占位保证与 motions 等长

## 边界

- 不实现「跳过失败继续播」
- 不在本轮下沉 `MotionPlanSession`
- PathPlan / 轨迹编辑管线独立，本共识不改其 Apply 语义
