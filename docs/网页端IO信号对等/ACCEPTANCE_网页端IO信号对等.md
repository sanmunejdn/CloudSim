# ACCEPTANCE — 网页端 IO 信号对等

| 项 | 结果 |
|----|------|
| LeftDock「信号」Tab：名称/类型/端口/值/强制；添加/删除/重置默认 | 已实现 `SignalsPanel` |
| 工程保存/打开恢复 `ioSignals` | Gateway save merge + open put |
| SET_DO / IF 等 `signalName` 下拉（DI/DO/AO） | PropsPanel + `/api/io/signals/names` |
| 改 signalName 后 `ioPort` 解析 | `HeadlessInstructionPropertyDelegate` |
| Debug\|x64 + Release\|x64：WebGateway / HostHeadless / CloudSimWeb | 通过（2026-08-14） |
| web UI `build:debug` + `build:release` → `bin\x64d\web` / `bin\x64\web` | 通过 |

## 未纳入本轮

- `/api/robot/run` 完整 Executor / 真 PLC
