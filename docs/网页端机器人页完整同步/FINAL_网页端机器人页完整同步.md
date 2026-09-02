# FINAL_网页端机器人页完整同步

桌面机器人坞能力已按 CONSENSUS 补齐到网页端：程序编辑栈、组/程序 CRUD、碰撞设置与规划确认、IK Seed、外轴页签、通讯降级说明。

## 交付摘要

| 层 | 内容 |
|----|------|
| Host | `HeadlessProgramEditBridge`、`HeadlessRobotCollisionBridge`、外轴 GET/PUT、playback `seedPolicy`、`robotCollision` 持久化 |
| Gateway | `/api/robot/program-edit/*`、`programs/crud`、`collision-*`、`external-axes` |
| Web UI | `InstructionPanel`（undo/组/程序/Seed）、`CollisionPanel`、`ExternalAxesPanel`、`CommPanel`、`RightDock` 页签 |

## 编译

- Debug\|x64 → `bin\x64d\`
- Release\|x64 → `bin\x64\`

## 已知降级

见 `TODO_网页端机器人页完整同步.md`。
