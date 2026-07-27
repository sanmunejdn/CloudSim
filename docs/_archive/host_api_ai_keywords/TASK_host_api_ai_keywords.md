# TASK — Host 公共接口接入 AI

## T1 文档与 Catalog
- 输入：DESIGN 按钮表
- 输出：6A 文档、`full_api_catalog.json`、`apiCatalogJson()` 同步
- 验收：条目含 keywords，与按钮文案一致

## T2 域与 Handler
- 输出：`geometry.ops`/`feature.build`/`labeling.annot` ids；5 个域注册；ai_config；AiWidget combo
- 验收：域下拉可见，registerBuiltinDomains 含新域

## T3 Keyword Matcher + Router
- 输出：`AiCatalogKeywordMatcher`；parseUserTextWithRules 接入；Router 按关键词路由
- 验收：「体素下采样」→ pointcloud.ops ActionPlan

## T4 选中桥
- 输出：`IPluginMainWindowHost::selectedBackendId` + MainWindow 实现；Executor 解析
- 验收：树选中对象可被 API 使用

## T5 Executor 分发
- 输出：`AiHostButtonApiDispatch` + QEventLoop；点云/几何/特征/标注/导入
- 验收：有选中时体素下采样真正调用 Host

## T6 DEVELOPER_GUIDE + 验收文档
- 输出：两份 GUIDE、ACCEPTANCE/FINAL/TODO
