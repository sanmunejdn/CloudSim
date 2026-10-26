# 仿真事件模式（薄 CEE）

桌面主程序：`TimeBased`（程序回放）与 `EventBased`（进程内薄 CEE）可切换。CEE 为事件模式逻辑权威，复用 `SimulationKernel` / `IoSignal`；外接 PLC 见下文。

| 文档 | 说明 |
|------|------|
| [实现说明.md](实现说明.md) | TimeBased / EventBased 与 Kernel 接线 |
| [外接PLC.md](外接PLC.md) | 外接 PLC（阶段 B）实现逻辑 |

相关：#68（CEE）· #69（外接 PLC）
