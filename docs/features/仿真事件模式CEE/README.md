# 仿真事件模式（薄 CEE）

桌面主程序：`TimeBased`（程序回放）与 `EventBased`（进程内薄 CEE）可切换。阶段 A：CEE 为事件模式唯一逻辑权威，复用 `SimulationKernel` / `IoSignal`；不做外接 PLC、Appearance、网页对等。

| 文档 | 说明 |
|------|------|
| [ALIGNMENT](ALIGNMENT_仿真事件模式CEE.md) | 对齐 |
| [CONSENSUS](CONSENSUS_仿真事件模式CEE.md) | 共识与验收 |
| [DESIGN](DESIGN_仿真事件模式CEE.md) | 设计 |
| [TASK](TASK_仿真事件模式CEE.md) | 任务拆分 |
| [ACCEPTANCE](ACCEPTANCE_仿真事件模式CEE.md) | 阶段 A 验收 |
| [CONSENSUS_B](CONSENSUS_阶段B_外接PLC.md) | 外接 PLC 共识 |
| [DESIGN_B](DESIGN_阶段B_外接PLC.md) | 外接 PLC 设计 |
| [TASK_B](TASK_阶段B_外接PLC.md) | 外接 PLC 任务 |
| [ACCEPTANCE_B](ACCEPTANCE_阶段B_外接PLC.md) | 外接 PLC 验收 |

BugTracker：#68（A）· #69（B）
