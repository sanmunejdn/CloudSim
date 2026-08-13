# ALIGNMENT — 轨迹离散 AI 确认对话框

## 原始需求

特征选定后不再静默按默认策略离散；AI 反馈需加载的离散策略与参数，并支持修改已作用的离散参数与管线算子。确认经模态对话框；确认后按用户选择执行整套离散+算子流程。

## 项目上下文

- 领域 `trajectory.feature`：识别 → 选编号 → `commitAiTrajectoryFeatures`
- 离散真源：`FeatureListDocument` / `discretizers/*.json`
- 管线真源：`TrajectoryOpDescriptor` + `ProcessFlowPresets.json`
- AI 经 `IAiAssistantHost` + `IPluginHostContext`（非 MCP）

## 边界

| 纳入 | 排除 |
|------|------|
| CAD `trajectory.feature` 首次确认 | Mesh 轨迹页 |
| 绑定 PathPlan 再编辑（revise） | 把算子塞进 Dock `AiConfirmPanel` |
| 模态对话框确认策略/参数/算子 | MCP |

## 关键决策

- A+B：首次确认 + 已作用再编辑
- 算子级：与轨迹编辑页同级
- 弹出对话框：复用 `FeatureDiscretizerParamPanel` / `TrajectoryPipelineListWidget` / `TrajectoryOpParamPanel`
