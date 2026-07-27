# TODO — Agent Runtime

## 冒烟

1. 连发两条指令 / 面板取消 → 不应长期 busy
2. 「体素下采样」确认执行；「体素下采样和ICP配准」连续两步
3. 几何识别 → 面板「确认创建」；轨迹 → 「确认并离散」/「重新识别」
4. 换文档后预填对象不串（`ai_agent_memory.json` → `prefs_by_doc`）
5. 查看 exe 旁 `ai_agent_trace.jsonl`

## 配置示例（ai_config.json）

```json
"agent": {
  "max_steps": 8,
  "auto_execute_low_risk": true,
  "enable_trace": true
}
```

## 可选后续

- ConfirmPanel 直接消费 JSON Schema（不止 args_schema）
- ToolResult.newBackendIds 由 Dispatch 填实
- compose 提案后 validatePlanJson
