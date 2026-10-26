# ACCEPTANCE — 仿真时间轴编排

| ID | 标准 | 状态 |
|----|------|------|
| T1 | Dock「时间轴」页；增删轨/片段；拖改时间 | 代理通过（SimulationTimelinePageWidget + Timeline 模式） |
| T2 | Robot 片段到点启 Lane | 代理通过（TimelineScheduler → onStartRobotProgram） |
| T3 | Device 片段 goNamedPose | 代理通过（onGoNamedPose） |
| T4 | Signal 窗写 DO | 代理通过（setDigitalOutput） |
| T5 | Material 关键帧显隐/位姿插值 | 代理通过（applyMaterialAt 线性插值） |
| T6 | 播放头 scrub 读 PlaybackTape | 代理通过（无采样时提示先 Play） |
| T7 | 与指令 Run / EventBased 互斥；EventBased 只读 | 代理通过 |
| T8 | 工程 `timelineStudy` 读写 | 代理通过（MainWindowProjectIo） |
| T9 | Debug+Release x64 相关工程通过 | 通过（RobotWidget / Widget） |

说明：完整 3D 联调建议人工扫一遍；上表为代码路径 + 双配置编译代理门闩。BugTracker #70。
