# ALIGNMENT — 网页端机器人信号页/指令/逻辑对等

## 需求

同步桌面端机器人命名 IO：信号页、指令 `signalName` 绑定、端口解析逻辑；工程 `ioSignals` 存取。

## 边界

| 做 | 不做 |
|----|------|
| LeftDock「信号」页 CRUD + 值/强制（仿真内存） | 真机 Plc / Cross Connection |
| GET/PUT `/api/io/signals` + names；工程开/存 `ioSignals` | 完整 `/api/robot/run` Executor（run 仍 stub） |
| Headless 改 `signalName` 时 resolve → `ioPort` | 改桌面 IoSignalPageWidget |

## 真源

桌面：`NamedSignalTable` + `IoSignalPageWidget` + `InstructionPropertyPanel` 信号 Combo。
