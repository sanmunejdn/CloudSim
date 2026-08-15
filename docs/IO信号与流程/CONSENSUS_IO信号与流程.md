# CONSENSUS_IO信号与流程

## 验收标准

1. 属性「信号」页按 Owner（机器人/自定义设备）编辑自持表；工程保存重开恢复
2. SET_DO / IF(Io) 使用**当前机器人**信号表
3. 连接站可 DO→DI 接线；改源 DO 后目标 DI 与设备指令上升沿正确
4. 面板可强制 DI（目标被 Force 时接线不覆盖）
5. Debug|x64 与 Release|x64：Data、RobotScene、RobotWidget、Widget 通过
6. 不做真机 Modbus/EIP、AI/AO 接线、Web 对等改造、旧 `ioSignals` 迁移

## 技术约束

- 侧车键：`backend_type::kProjectKeyIoSignalNetwork`（`ioSignalNetwork`）
- 设备表：`CustomDeviceBackendData::signals`
- 机器人表：侧车 `owners[sceneBackendId].signals`
- Sink 后端：`RobotIoSinkBackend::{Simulation, PlcStub}`
- 指令 JSON：`signalName` 可选；优先解析到当前机器人 port
