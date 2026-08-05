# FINAL — 工艺流程二期工业加深

## 已交付
- 班次日历：`ShiftCalendar` + DES `DispatchWake`（班次外不新开加工）
- 到达：`arrivalMode=fixed|exponential` + warmup UI
- 绑定 UI：属性面板 `bindingBackendId/programId`（DES 仍可走 Preview 记录）
- 导出：CSV 含缓冲段

## 验收
启用 08:00–16:00 班次后吞吐低于连续 24h；指数到达 WIP 波动可见；绑定可存入工程 JSON。
