# CONSENSUS_IO信号与流程

## 验收标准

1. 左侧「信号」Tab 可增删命名 DI/DO，工程保存重开恢复
2. SET_DO / IF(Io) 可选信号名；运行时与面板一致
3. 面板可强制 DI，IF 随之分支
4. DEVICE_AXIS 运行后自定义设备位姿正确
5. Debug|x64 与 Release|x64：RobotScene、RobotWidget、Widget 通过
6. 不做真机 Modbus/EIP、Cross Connection、流程图编辑器、Web

## 技术约束

- 侧车键：`backend_type::kProjectKeyIoSignals`（`ioSignals`）
- Sink 后端枚举：`RobotIoSinkBackend::{Simulation, PlcStub}`（PlcStub 空接线）
- 指令 JSON：`signalName` 可选；优先解析到 port
