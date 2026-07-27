# TODO：机器人指令执行（后续）

- [ ] 将链式规划 / Cache / 失败策略下沉为 `MotionPlanSession`（RobotScene）
- [ ] PTP 梯形/S 曲线时间参数化（臂 + 外轴统一剖面）
- [ ] LINE blend 圆弧过渡
- [ ] Deprecate `RobotInstructionPlaybackEngine`
- [ ] 轨迹 `ReachabilityFilter` 接入 `Controller::plan`
- [x] Run 保留 `jointTrajectoryRad` + 外轴段内插值（见 `docs/外部轴联动求解/`，2026-07-23）
