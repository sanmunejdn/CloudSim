# FINAL — Agent Runtime 框架优化（ToolResult / FSM / 统一 Session）

## 本轮交付

1. **ToolResult + 状态机**：`AiToolResult`、`AiAgentState`；Runtime `Idle→Proposing→AwaitingConfirm→Executing→Done`；新消息 `cancelTurn` 清 busy
2. **统一 Confirm Session**：`beginDomainConfirmAsync`；识别/轨迹不再 LocalConfirm
3. **多轮 tool messages**：`chatWithTools` + `appendToolObservation`；观测回灌 LLM
4. **JSON Schema 单源**：`AiArgsSchema`；记忆 `prefs_by_doc[documentId]`
5. **Trace + 配置**：`ai_agent_trace.jsonl`；`ai_config.agent.{max_steps,auto_execute_low_risk,enable_trace}`

SDK ABI：`0x00010200`

## 验证

- 连发消息 / 取消不卡 busy
- 识别「确认创建」、轨迹「确认并离散 / 重新识别」走 Host Session
- 「先下采样再 ICP」（LLM）依赖 messages 回灌
- 换文档预填不串（prefs_by_doc）
- 失败步可在 exe 旁 `ai_agent_trace.jsonl` 复盘
