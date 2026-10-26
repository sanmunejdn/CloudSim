# CONSENSUS — 阶段 B：外接 PLC 逻辑权威

## 1. 锁定决策

| 项 | 约定 |
|----|------|
| EventBased 逻辑权威 | `localCee` \| `externalPlc`（互斥） |
| 通讯 | PlcCommSDK，首期 Modbus TCP；bool only |
| ExternalPlc | **只镜像**信号；不求值 CEE；仍禁 DevicePoseSignalDriver |
| PLC→DI | swapAll 后写 forced DI；WAIT 再下一拍可见 |
| 读写线程 | `readTag`/`writeTag` 仅工作线程 |
| 断线 | 警告 + 仿真继续 |
| Run 中切权威 | 拒绝 |

## 2. 验收

见 ACCEPTANCE_阶段B_外接PLC.md（B1–B6）。

BugTracker：#69
