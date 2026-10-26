# 仿真外置控制器

Webots 风格：独立控制器进程经 localhost TCP（**19620**）驱动仿真关节。协议已冻结。

## 文档索引

| 文档 | 说明 |
|------|------|
| [协议.md](协议.md) | **协议冻结**（schema / 端口 / 错误码）— 禁止私改 |
| [实现说明.md](实现说明.md) | 架构、STEP 数据流、模块落点、异常策略 |
| [P0_语义附录.md](P0_语义附录.md) | 无 STEP / 断连 / actualJointRad / dtMs / 限幅 |
| [P3_立项说明.md](P3_立项说明.md) | STEP_POSE 草案（已由 P5 落地） |
| [P4_设备语义增量.md](P4_设备语义增量.md) | 限幅、sensors、环境变量、BAD_DT |
| [P5_STEP_POSE.md](P5_STEP_POSE.md) | 笛卡尔 STEP_POSE + Host IK |
| [P6_多会话.md](P6_多会话.md) | 多实例端口 19620+i |

## 快速要点

- Transport：`127.0.0.1:19620`（避开 RobotComm `19610`）
- 帧：一行一条 JSON + `\n`，`protocolVer=1`
- 模式：`ExternalController` 与内置指令回放互斥；默认关闭
- 桌面：指令页勾选「外置控制器」或 **设置…** → 对话框选语言、看源码、运行样例；**打开控制器目录** → `resource/controllers/`
- 构建：Debug\|x64 → `bin\x64d\`；Release\|x64 → `bin\x64\`
