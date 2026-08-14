# CONSENSUS — 网页端机器人信号页/指令/逻辑对等

## 验收

1. LeftDock 有「信号」Tab：名称/类型/端口/值/强制；增删/重置默认
2. 工程保存重开恢复 `ioSignals`
3. SET_DO / IF 等指令属性 `signalName` 为下拉（按 DI/DO/AO）
4. 改 signalName 后 `ioPort` 同步解析
5. Gateway + Web：Debug\|x64 与 Release\|x64 相关工程通过（Web PostBuild 可用 SKIP）

## 技术

- `DocumentHost::namedSignalTable()` 为定义真源
- Gateway 仿真运行时值与强制存进程内表（对齐 Sink 面板能力，不链 RobotWidget）
- React：`SignalsPanel` + `ioSignals` API + `instrPropView` 动态 options
