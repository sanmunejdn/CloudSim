# FINAL — 轨迹离散 AI 确认对话框

## 交付摘要

- 新增 `TrajectoryPlanConfirmDialog`：策略、离散参数、管线算子同屏模态确认。
- Host ABI 1.51.0（`0x00013300`）：`proposeAndConfirmTrajectoryPlan` / `loadBoundTrajectoryPlanForAi` / `reviseAiTrajectoryPlan`。
- `commitFeaturePlanFromAi` 优先应用 `pipeline[]`，否则 recipe 回退。
- Coordinator：选特征后弹对话框；「修改离散参数/改管线算子」走 revise。

## 编译

- Debug|x64：CloudSimPluginSDK、RobotWidget、Widget、AiWidget 通过
- Release|x64：同上通过

## 文档

`docs/轨迹离散AI确认对话框/` 下 ALIGNMENT / CONSENSUS / DESIGN / TASK / ACCEPTANCE / FINAL / TODO
