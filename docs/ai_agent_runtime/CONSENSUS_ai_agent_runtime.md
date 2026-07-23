# CONSENSUS — Full AI Agent Runtime

## 验收标准

1. 点云/导入/几何/特征/标注/网格等需参命令均出 Dock 面板
2. 体素/随机/球裁剪/ICP·SPARE/导入导出字段正确
3. 「先…再…」可连续两次面板确认
4. rules 与 LLM `tool_calls` 共用面板
5. 快照填对象下拉；记忆预填上次参数
6. high/medium 有风险提示；取消不执行
7. 无整窗模态主路径；识别/轨迹并入统一面板
8. 两份 DEVELOPER_GUIDE + 本目录 6A 齐全

## 技术方案

- `IAiAssistantHost::runAgentTurnAsync` / `submitAgentConfirm` / `cancelAgentConfirm` / `cancelAgentTurn`
- `AiAgentRuntime`：enqueue 提案 → NeedConfirm → 执行 → StepDone → 可选多步
- `AiLlmClient::buildOpenAiToolsFromCatalog` + `chatWithTools`
- Catalog `args_schema` 动态生成表单

## 约束

- SDK 版本 `0x00010100`（vtable 追加）
- `AiAgentPickDialog` 仅非 Agent 遗留兜底
