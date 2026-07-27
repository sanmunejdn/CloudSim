# FINAL — Host 公共接口接入 AI

## 交付摘要

将 PointCloud / Geometry / Feature Build / Labeling Dock 的调用按钮，以 **按钮中英文名 = keywords** 接入 AI 助手完整竖切：

- Catalog + `AiCatalogKeywordMatcher`（最长匹配）
- 5 个 Catalog ActionPlan 域注册 + AiWidget / `ai_config.defaults.json`
- `AiHostButtonApiDispatch` 真调 Host（异步 QEventLoop）
- **Agent 对话框**（`AiAgentPickDialog`）：缺 backend/path 时枚举对象或选文件，而非仅报错
- 选中桥：`IPluginMainWindowHost::selectedBackendId` → `PluginHostContext::selectedBackendId`
- 文档：ALIGNMENT/CONSENSUS/DESIGN/TASK/ACCEPTANCE + 两份 DEVELOPER_GUIDE

## 关键源文件

| 文件 | 作用 |
|------|------|
| `AiApiCatalogEmbedded.*` | 嵌入 Catalog |
| `AiCatalogKeywordMatcher.*` | 按钮名 rules |
| `CatalogActionPlanDomainHandler.*` | 通用域 Handler |
| `AiHostButtonApiDispatch.*` | API 分发 + Agent 对话框 |
| `AiAgentPickDialog.*` | 源/目标/文件选择 UI |
