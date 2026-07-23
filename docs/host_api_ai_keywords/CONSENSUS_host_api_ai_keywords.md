# CONSENSUS — Host 公共接口接入 AI

## 需求与验收

1. 输入精确按钮中文/英文名 → rules 命中 → 执行对应 Host API
2. AiWidget 可选新域；auto 按关键词路由
3. 无选中/缺 path 时可读错误、不崩溃
4. 交互类进入 pick；取消有明确失败摘要
5. Catalog 与 `full_api_catalog.json` 同步；两份 DEVELOPER_GUIDE 更新

## 技术方案

- Catalog 条目：`id`（Host 方法名）+ `domains` + `keywords[]` + `summary`
- `AiCatalogKeywordMatcher`：最长关键词优先 → ActionPlan v2
- `CatalogActionPlanDomainHandler`：按 domain 校验 api 归属后执行
- `AiHostButtonApiDispatch`：Executor 扩展分发（含 QEventLoop）
- `IPluginMainWindowHost::selectedBackendId()` 供 Executor 解析对象

## 约束

- 不改无按钮的 Host ABI 中间插入；新能力挂在 MainWindowHost / 执行器侧
- PointNet 预标注：无单一 Host 方法，执行时返回引导用标注面板的错误信息
