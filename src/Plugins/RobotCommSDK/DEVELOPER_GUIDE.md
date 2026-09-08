# RobotCommSDK

> **文档导航**：[全库入口](../../../../docs/README.md) · [全量目录](../../../../docs/全量目录.md) · [开发手册](../../../../docs/开发手册/01-总览.md) · [产品索引](../../../docs/README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

C++ 客户端 DLL：经 localhost TCP JSON 连接 `RobotCommBridge`，拉取真实机器人关节角 / 末端位姿。

| 项 | 值 |
|----|------|
| 输出 | `RobotCommSDK.dll` |
| 定义 | `ROBOTCOMM_SDK_LIB` |
| 依赖 | Winsock2、nlohmann/json（`bin/SDK/JSON`） |

对外接口：`IRobotMotionClient` / `createRobotMotionClient()`。
