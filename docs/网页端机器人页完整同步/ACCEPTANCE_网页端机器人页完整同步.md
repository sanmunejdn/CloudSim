# ACCEPTANCE_网页端机器人页完整同步

- [x] 指令：undo/redo、建组/解散/改名、程序新建/改名/删除
- [x] 指令：IK Seed 链式/当前，影响 plan 与 Run
- [x] 碰撞：设置读写落盘；选起终点规划；确认插入轨迹
- [x] 外轴：读写配置并进规划上下文
- [x] 通讯：页签可连 Bridge/机器人或明确错误
- [x] CloudSimWeb Debug|x64 + Release|x64

## 验证记录

| 项 | 结果 |
|----|------|
| Debug\|x64 `CloudSimWeb` | 通过（`bin\x64d\CloudSimWeb.exe` + `bin\x64d\web`） |
| Release\|x64 `CloudSimWeb` | 通过（`bin\x64\CloudSimWeb.exe` + `bin\x64\web`，约 2026-08-31 23:16） |
| 通讯 | 页签说明依赖 `RobotCommBridge`；无 Host `/api/robot/comm/*` 时不提供假成功 |
| 碰撞规划 | JointLerp+FK 路径（非桌面 OMPL 预览）；确认走 `insertRawTrajectoryBetween` |
