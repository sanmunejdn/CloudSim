# FINAL — 机器人通讯

## 交付摘要

实现 CloudSim 真实机器人**只读反馈 + 数字孪生镜像**链路：

```
RobotCommPageWidget → RobotCommSDK (TCP JSON) → RobotCommBridge (HSL) → ABB/Fanuc/KUKA
                                      ↓
                         applyJointAnglesRad → 场景镜像
```

## 主要产物

| 路径 | 说明 |
|------|------|
| `scripts/fetch_hslcommunication.ps1` | 拉取 HSL 12.9.1 |
| `bin/SDK/HslCommunication-12.9.1/` | HSL + Json 运行时 |
| `CloudSim/src/Plugins/RobotCommSDK/` | C++ 客户端 DLL |
| `CloudSim/tools/RobotCommBridge/` | C# Bridge（Fanuc/KUKA/ABB Adapter） |
| `RobotCommPageWidget` | 仿真 Dock 新 Tab |
| `docs/机器人通讯/` | 6A 文档 |

## 参考

- HSL Demo：`c:\CODE\HslCommunication-master`
- robdts HSLCLI：Fanuc `D751`/`D777`、KUKA `KukaTcpNet`
