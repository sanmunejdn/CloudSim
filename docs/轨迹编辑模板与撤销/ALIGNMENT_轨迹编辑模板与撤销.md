# ALIGNMENT · 轨迹编辑模板与撤销

## 原始需求

整理轨迹编辑页流水线保存/加载与撤销/重做逻辑并完善；轨迹生成（CAD）离散策略与参数增加模板保存/加载。

## 项目上下文

- 轨迹编辑：`TrajectoryEditPageWidget` + `TrajectoryEditSession` + `ProgramEditService`
- 现流水线模板：单槽 `QSettings("CloudSim","TrajectoryPipeline")/pipelineJson`
- 撤销：仅程序 Command 栈；草稿算子编辑不可撤销
- CAD 离散：`FeatureTrajectoryPageWidget` + `FeatureDiscretizerParamPanel`，无用户模板

## 边界确认

| 纳入 | 不纳入 |
|------|--------|
| 草稿流水线独立撤销栈 | Mesh 轨迹参数模板 |
| 命名多模板库 + 导入/导出 | 改造 `ProcessFlowPresets.json` |
| 修 Apply/Undo 与 PathPlan 不同步 | 草稿期仍直写 PathPlan |
| CAD 离散 strategyId+params 命名模板 | 模板内保存几何/featureId |

## 已确认决策

- 撤销按钮：优先草稿栈，空则程序栈
- 草稿期不调用 `syncPipelineToBoundPathPlan`
- 模板存 `%AppData%/CloudSim/templates/{pipeline|discretize}/`
