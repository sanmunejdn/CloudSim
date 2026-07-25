# DESIGN — 工序排程 DES 引擎

## 三件套

| 层 | 实现 |
|----|------|
| DES | `DesEngine` + 优先队列事件 |
| 调度 | `IDispatchPolicy`：FIFO / SPT；`IScheduler` 空头预留 CP/RL |
| 可视化数据 | `SimStatistics` + `OperationTrace`（甘特数据源，表格展示） |

## 双层模型

- **PlantGraph**：画布 `processFlow` 拓扑
- **JobSet**：手工 JSON 或沿 start→end 路径自动生成 Op 序列

## 运行

菜单 / 右侧报表面板 → `ProcessFlowSimController` → `enqueueJob` 后台 `runUntil(horizon)` → UI 回填报表。

## 预留

- `IStationExecutor` / `NullStationExecutor` / `StationBinding`
- `IScheduler`（静态 CP 日程）
- 甘特控件、画布 token 动画、机器人 Host ABI：后期
