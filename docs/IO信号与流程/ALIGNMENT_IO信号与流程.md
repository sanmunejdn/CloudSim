# ALIGNMENT_IO信号与流程

## 原始需求

左侧 Property Dock 新增与「设备」同级的「信号」页：命名 IO 表供程序 DO/DI/条件绑定；新增自定义设备轴运动指令；真机 Bridge/PlcComm 仅预留。

## 边界

- 扩展现有指令树，非整图编辑器
- 仿真内存信号表本期验收；真机不验收
- 不做 Cross Connection 逻辑门、Web、独立流程图

## 需求理解

- `NamedSignalTable` + `NamedSignalIoSink(IRobotIoSink)` + 工程侧车 `ioSignals`
- `IoSignalPageWidget` 挂载 Property Dock
- SET_DO/AO 与 IF/WHILE Io 绑定 `signalName`
- `DeviceAxis` 指令经 `CustomDeviceKinematics::applyQ`

## 疑问澄清

- 已锁定：流程形态 / 运行时 / 页面位置（见计划）
