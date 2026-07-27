# ACCEPTANCE — 机器人通讯

## 检查项

| 项 | 状态 | 说明 |
|----|------|------|
| `scripts/fetch_hslcommunication.ps1` | 通过 | 已落盘 `bin/SDK/HslCommunication-12.9.1/`（含 net451 DLL + Newtonsoft.Json） |
| RobotCommSDK 编译 | 通过 | `bin/x64d/RobotCommSDK.dll` |
| RobotCommBridge 编译 | 通过 | `bin/x64d/RobotComm/RobotCommBridge.exe`（net472） |
| RobotWidget 通讯 Tab | 通过 | Dock `kTabIndexRobotComm=7`，「机器人通讯」页 |
| 6A 文档 | 通过 | ALIGNMENT / CONSENSUS / DESIGN / TASK 已写 |
| 真机联调 | 待现场 | 需 Bridge 进程 + 机侧选项（RWS / Fanuc 以太网 / KukaTcp） |
| HSL 商用授权 | 待配置 | 开发可无码（约 8h）；交付需 `HSL_AUTH_CODE` |

## 本地冒烟（无真机）

1. 启动 Bridge：`bin\x64d\RobotComm\RobotCommBridge.exe --port 19610`
2. 启动 CloudSim，打开机器人 Dock →「机器人通讯」
3. Bridge `127.0.0.1:19610`，点连接（无真机时期望机器人连接失败，但 Bridge `ping`/`connect bridge` 应成功）
