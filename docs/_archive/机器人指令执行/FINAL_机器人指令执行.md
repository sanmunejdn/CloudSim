# ACCEPTANCE / FINAL：机器人指令执行

## 已实现

| 项 | 状态 | 位置 |
|----|------|------|
| 部分规划失败仍播前缀 | 完成 | `RobotSimulationController::onSimulationStartTriggered` |
| 播到失败点前停止 | 完成 | `RobotProgramExecutor::startMotionSegment` + tick `Aborted` |
| 播放起点修正 | 完成 | `playbackStartAngles` |
| 轨迹优先插值 | 完成 | `tickMotionSegment`；Run **保留** `jointTrajectoryRad` |
| LINE 笛卡尔采样 IK | 完成 | `LinePlanner::plan` |
| 外轴段内插值 | 完成 | `motionSegmentProgress01` + `applyExternalAxisFromPlan` lerp |
| 地轨行程计入时长 | 完成 | `externalAxisTravelDurationSec` / `applyLastIkExternalAxisToPlan` |
| 开发者文档同步 | 完成 | RobotScene / RobotWidget DEVELOPER_GUIDE |
| Run 懒规划（急算前缀 + 段前补算 + lookahead 回写） | 完成 | `ensurePlaybackPlansReady` / `syncPlanMotionAtIndex` / `updateMotionPlanResult` |
| 万级播放流畅（O(1) 种子、有界 Cache、仅 pending sync） | 完成 | `m_playbackRollingSeedQ` / `PlanResultCache` / `playbackPlanLite` |

## 手工验收

- [ ] 10 点程序仅末 2 点 IK 失败 → 前 8 段可播，随后告警停止
- [ ] 第 1 点失败 → 不启动
- [ ] 全成功 → 行为与改前一致（从程序起点播放）
- [ ] URDF LINE 路径近似直线（相对改前关节插值更贴直线）
- [ ] 数百/数千路点 Run 启动不卡死（仅急算前 16 段）；播放流畅，段前偶发短卡可接受
- [ ] 懒规划中途某点 IK 失败 → 播到该点前停止并告警
- [ ] 启用地轨的点云/多点 Run：臂与滑轨连续运动，非一瞬间就位

## TODO（后续）

- MotionPlanSession 下沉 Scene 层
- PTP/LINE 梯形时间剖面与 tick 对齐（臂+外轴统一剖面）
- 清理遗留 `RobotInstructionPlaybackEngine`
- `ReachabilityFilter` 接真 IK
