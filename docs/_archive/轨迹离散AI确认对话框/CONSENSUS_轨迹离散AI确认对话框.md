# CONSENSUS — 轨迹离散 AI 确认对话框

## 需求与验收

1. 选特征后弹出对话框：策略、离散参数、完整算子列表可增删改参；确认后 raw/管线与对话框一致。
2. 取消/重新识别不写坏 PathPlan。
3. 已离散 PathPlan 可再编辑：同对话框加载当前 `sourceFeatureJson`+ops → 确认后重离散并更新算子。
4. Debug|x64 与 Release|x64 相关工程编译通过。
5. 无 `pipeline[]` 的旧 plan 回退 `applyRecipePresetByKind`。

## 技术方案

- Plan JSON v2：`mode`、`features[].strategyId/params`、`pipeline[]` 权威；`suggestedPipelineTemplate` 仅作展开种子。
- `TrajectoryPlanConfirmDialog`（RobotWidget）：模态确认。
- Host：`proposeAndConfirmTrajectoryPlan` / `loadBoundTrajectoryPlanForAi` / `reviseAiTrajectoryPlan`；改造 `commitAiTrajectoryFeatures`。
- Coordinator：TrajectoryCommit 只弹该对话框一次，跳过空 `AiConfirmPanel`。

## 约束

- 不引入 MCP；继续 Host 桥。
- ABI：`IPluginHostContext` 仅末尾追加虚函数。
