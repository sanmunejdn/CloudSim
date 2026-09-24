# 仿真外置控制器

Webots 风格：独立控制器进程经 localhost TCP（**19620**）驱动仿真关节。协议已冻结。

## 文档索引

| 文档 | 说明 |
|------|------|
| [CONSENSUS_仿真外置控制器.md](CONSENSUS_仿真外置控制器.md) | **协议冻结**（schema / 端口 / 错误码）— 禁止私改 |
| [ALIGNMENT_仿真外置控制器.md](ALIGNMENT_仿真外置控制器.md) | 需求对齐、边界、与 RobotComm 上下文 |
| [DESIGN_仿真外置控制器.md](DESIGN_仿真外置控制器.md) | 架构、STEP 数据流、模块落点、异常策略 |
| [TASK_仿真外置控制器.md](TASK_仿真外置控制器.md) | 原子任务 G0 / P1 A–D / P2 E–G |
| [ACCEPTANCE_仿真外置控制器.md](ACCEPTANCE_仿真外置控制器.md) | 验收清单、手工步骤、已知限制 |
| [P0_语义附录.md](P0_语义附录.md) | 无 STEP / 断连 / actualJointRad / dtMs |
| [P3_立项说明.md](P3_立项说明.md) | 笛卡尔 IK / 多会话 / ROS 草案 |

## 快速要点

- Transport：`127.0.0.1:19620`（避开 RobotComm `19610`）
- 帧：一行一条 JSON + `\n`，`protocolVer=1`
- 模式：`ExternalController` 与内置指令回放互斥；默认关闭
- 桌面：指令页勾选「外置控制器」或点 **设置…** → 对话框选语言、看源码、运行 Python/C++ 样例；**打开控制器目录** 对应 `resource/controllers/`
- 构建：Debug\|x64 → `bin\x64d\`；Release\|x64 → `bin\x64\`

权威顺序：活 `CloudSim/docs` > [`ARCHIVE_ZIP_LOCATION.txt`](../../ARCHIVE_ZIP_LOCATION.txt) 指向的归档 zip。
