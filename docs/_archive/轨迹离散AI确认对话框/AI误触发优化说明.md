# AI 误触发与智能度优化

## 本轮已落地

### Agent
- `auto_execute_low_risk` **默认 false**（低风险也不自动跳过确认）
- `require_keyword_hit` **默认 true**：Catalog 无可靠关键词命中时，**不**调用 LLM `tool_calls`，直接澄清
- 同时禁止 LLM 编造多步 plan（场景/工艺规则、多 keyword 串联仍可用）

### Catalog 关键词
- 最短长度：中文 ≥2，拉丁 ≥3
- 拉丁词要求左右非字母数字边界，降低 `ICP`/`Move` 类误命中

### 意图分类
- `AiIntentClassifier::classifyByRules`：打分选域（`min_score`，默认 2）
- `router.mode=local_classify`：规则失败时再用本地小模型选域（需 `local_model`）
- 仍失败 → Coordinator 澄清、不执行

## `ai_config.json` 关键字段

```json
"router": { "mode": "rules_score", "min_score": 2, "local_model": "qwen2.5:3b", "base_url": "http://127.0.0.1:11434/v1" },
"agent": { "auto_execute_low_risk": false, "require_keyword_hit": true }
```

`mode` 可选：`rules_score`（默认逻辑）| `local_classify`（规则空再 LLM）| `explicit_ui`（行为同 rules_score，需手动选域更稳）

## 使用建议

1. 领域下拉尽量选具体域；Auto 仅配合明确业务句。
2. Agent 类操作尽量说 Dock 按钮原文：「体素下采样」「点云匹配」。
3. 若要让 LLM 自由选工具：设 `require_keyword_hit: false`（风险更高）。
