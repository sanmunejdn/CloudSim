# CloudSimControllerSDK

> **文档导航**：[全库入口](../../../docs/README.md) · [仿真外置控制器](../../../docs/features/仿真外置控制器/README.md) · [CONSENSUS](../../../docs/features/仿真外置控制器/CONSENSUS_仿真外置控制器.md)

C++ 客户端 DLL：经 localhost TCP JSON（`127.0.0.1:19620`）连接 Host `ControllerManager`，驱动仿真关节。

| 项 | 值 |
|----|------|
| 输出 | `CloudSimControllerSDK.dll` → `bin\x64d\` / `bin\x64\` |
| 定义 | `CLOUDSIM_CONTROLLER_SDK_LIB` |
| 依赖 | Winsock2、nlohmann/json（`bin/SDK/JSON`） |

对外接口：`IControllerClient` / `createControllerClient()`。

与 [RobotCommSDK](../RobotCommSDK/DEVELOPER_GUIDE.md)（端口 19610、真机反馈）职责分离，禁止混用端口。
