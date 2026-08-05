# FINAL — 工艺流程三期执行与智能

## 已交付
- `PreviewStationExecutor`：记录绑定，DES 仍按节拍推进（`executorMode=drivePreview`）
- AGV 节点 kind：`agv`（按输送机建模，capacity=车队规模）
- `PriorityListScheduler` +「优化后仿真」按钮（推荐策略后跑 DES）

## 后续
- 真机 Preview/Run 需 Host Robot ABI
- CP-SAT / OR-Tools 可替换 `IScheduler` 实现
- AGV 路网与空驶矩阵可再加深
