# CONSENSUS_网页端机器人页完整同步

桌面 `RobotSimulationDockWidget` ↔ 网页右坞「设备 → 机器人」。

## 本轮补齐

| 项 | 做法 |
|----|------|
| 程序 undo/redo | Host `ProgramEditStack` |
| 指令组 CRUD | `Create/Remove/RenameInstructionGroupCommand` + API |
| 程序新建/改名/删除 | `RobotProgramCatalog` + UI |
| 碰撞设置 | `DocumentHost` 持久化 `robotCollision` |
| 碰撞规划/确认 | Headless 规划 + `insertRawTrajectoryBetween`；面板起终点 |
| IK Seed | plan/run 接受 `seedPolicy`；指令页 Chain/Current |
| 外轴 | GET/PUT `/api/robot/external-axes` + 页签 |
| 通讯 | 页签 + Host 桥（无 Bridge.exe 时明确报错，不假成功） |

## 仍降级

- 碰撞规划 OSG 预览路径（网页用指令插入替代画面黄线）
- 通讯依赖外部 `RobotCommBridge` 进程
