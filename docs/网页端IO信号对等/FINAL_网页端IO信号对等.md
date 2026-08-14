# FINAL — 网页端 IO 信号对等

网页 LeftDock 已对齐桌面命名 IO：信号表 CRUD、运行时值/强制、指令 `signalName` 枚举与端口解析，工程侧车键 `ioSignals`。

真源：`DocumentHost::namedSignalTable()`；仿真值在 Gateway 进程内 `g_ioRuntime`（不链 RobotWidget Sink）。
