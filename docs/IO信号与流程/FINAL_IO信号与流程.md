# FINAL_IO信号与流程

## 交付摘要

已实现命名 IO 信号表、仿真 Sink、Property Dock「信号」页、指令 `signalName` 绑定，以及 `DeviceAxis` 自定义设备轴指令。

## 编译

| 工程 | Debug\|x64 | Release\|x64 |
|------|------------|--------------|
| RobotScene | 通过 | 通过 |
| RobotWidget | 通过 | 通过 |
| Widget | 通过 | 通过 |

## 主要文件

- `RobotScene/NamedSignalTable.*`、`RobotProgramExecutor` DeviceAxis 分支
- `RobotWidget/NamedSignalIoSink.*`、`IoSignalPageWidget.*`
- `Widget` Dock Tab / 工程侧车 `ioSignals` / 属性 Combo
- `docs/IO信号与流程/`

## 说明

- Qt 宏冲突：避免在头文件中使用标识符 `signals`
- 真机 Plc/Bridge 仅 `RobotIoSinkBackend::PlcStub` 预留
