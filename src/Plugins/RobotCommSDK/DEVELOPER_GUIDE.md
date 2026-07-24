# RobotCommSDK

C++ 客户端 DLL：经 localhost TCP JSON 连接 `RobotCommBridge`，拉取真实机器人关节角 / 末端位姿。

| 项 | 值 |
|----|------|
| 输出 | `RobotCommSDK.dll` |
| 定义 | `ROBOTCOMM_SDK_LIB` |
| 依赖 | Winsock2、nlohmann/json（`bin/SDK/JSON`） |

对外接口：`IRobotMotionClient` / `createRobotMotionClient()`。
