# ALIGNMENT — Full AI Agent Runtime

## 原始需求

把「关键词 → ActionPlan →（模态缺参）→ 执行」升级为 Observe–Think–Act：场景快照 + 记忆 + rules/LLM tool_calls → Dock 内嵌 `AiConfirmPanel` → 确认后执行；凡带 `args_schema` 的 API 一律确认。

## 项目理解

- Catalog 真源：`tools/ai-training/catalog/full_api_catalog.json`（`risk` + `args_schema`）
- Host：`AiAgentRuntime` / `AiSceneSnapshotBuilder` / `AiAgentMemory` / `AiLlmClient::chatWithTools`
- UI：`AiConfirmPanel` 嵌于 `AiAssistantDockWidget`；Coordinator 统一 pending
- 识别/轨迹确认并入同一面板（遗留按钮仅兜底接线）

## 边界

- 主路径不阻塞整窗 `QDialog`
- Agent 执行 `tryExecute(..., allowModalDialogs=false)`
- 最多 8 步；新消息打断 `cancelAgentTurn`

## 已决歧义

| 点 | 决策 |
|----|------|
| 确认 UI | Dock 内嵌面板 |
| Runtime 落点 | Host 侧，Widget 只确认回调 |
| 识别/轨迹 | 同一 `AiConfirmPanel` |
