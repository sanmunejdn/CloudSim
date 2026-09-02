# ACCEPTANCE_网页端指令IK播放对等

- [x] Host `planMotionInstruction` 规划后做折圈；越限返回中文错误
- [x] `HeadlessRobotPlaybackBridge` 急算 16 段；播放 tick 补 `lazyPending`；失败停机
- [x] 网页预览/Run 不再用示教关节 CSV 短路
- [ ] 用同一工程在网页 Run：长程序能播过第 16 段；折圈失败停在该段并显示 abortSummary
- [x] Debug|x64 与 Release|x64 编译（RobotScene / Host / HostHeadless / RobotWidget / WebGateway / CloudSimWeb）
