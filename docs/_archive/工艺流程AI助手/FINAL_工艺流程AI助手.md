# FINAL — 工艺流程 AI 助手

## 交付摘要

在工艺流程模式下重新露出 AI 助手，并新增 Agent 域 `process.flow`：自然语言 →（规则/LLM）→ Host 桥接 → 整图写入 + 同步 DES。

## 关键改动

- `MainWindow::enter/exitProcessFlowSideUi`：tabify AI 与仿真 Dock
- `IProcessFlowAiBridge` + Host ABI **1.20.0**
- `ProcessFlowAiBridge` / Catalog 三工具 / `AiProcessFlowRules`
- 文档：`docs/工艺流程AI助手/`、插件 README

## 使用提示

- 需活动文档；插件 `minHostVersion` 1.20.0
- remote_llm 默认关闭；规则模板可在无云端时处理常见「N 工位流水线」口语
