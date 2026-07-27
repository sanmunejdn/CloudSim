# ACCEPTANCE — Full AI Agent Runtime

| 项 | 状态 |
|----|------|
| Catalog 全量 risk/args_schema + embed | 通过 |
| AiSceneSnapshotBuilder / AiAgentMemory | 通过 |
| AiConfirmPanel Dock 内嵌 | 通过 |
| Agent Runtime + tool_calls + cancelAgentTurn | 通过 |
| Agent 路径无模态补参 | 通过 |
| 识别/轨迹走统一面板 | 通过 |
| risk medium/high 提示 | 通过 |
| docs/ai_agent_runtime 6A | 通过 |
| AiSDK / Host DEVELOPER_GUIDE | 通过 |

手动冒烟建议：「体素下采样」「点云匹配」「先体素下采样再 ICP」/「体素下采样和ICP配准」；识别「确认创建」；轨迹「确认并离散」「重新识别」。

## 优化项（续）

| 项 | 状态 |
|----|------|
| 移除 Dock 遗留确认按钮 | 通过 |
| ConfirmPanel 自定义确认/次要按钮 | 通过 |
| 多步：剩余 keywords 继续 | 通过 |
| StepDone 观测摘要（参数+对象数） | 通过 |
