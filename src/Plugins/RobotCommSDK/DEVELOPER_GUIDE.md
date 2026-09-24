# RobotCommSDK

> **文档导航**：[全库入口](../../README.md) · [全量目录](../../README.md) · [开发手册](../../开发手册/01-总览.md) · [产品索引](../README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

C++ 客户端 DLL：经 localhost TCP JSON 连接 `RobotCommBridge`，拉取真实机器人关节角 / 末端位姿。

| 项 | 值 |
|----|------|
| 输出 | `RobotCommSDK.dll` |
| 定义 | `ROBOTCOMM_SDK_LIB` |
| 依赖 | Winsock2、nlohmann/json（`bin/SDK/JSON`） |

对外接口：`IRobotMotionClient` / `createRobotMotionClient()`。

## 与仿真外置控制器的边界

| | RobotCommSDK | CloudSimControllerSDK |
|--|--------------|------------------------|
| 默认端口 | `19610` | `19620` |
| 对端 | `RobotCommBridge`（真机/桥） | Host `ControllerManager`（仿真） |
| 语义 | 拉真实关节/位姿 | 推仿真 `targetJointRad`（STEP） |

二者协议字段与端口均独立；**禁止**在同一连接上混用帧类型。专题：[`docs/features/仿真外置控制器/`](../../../docs/features/仿真外置控制器/)。
