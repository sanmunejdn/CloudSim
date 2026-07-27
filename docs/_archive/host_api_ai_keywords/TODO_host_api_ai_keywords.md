# TODO — Host 公共接口接入 AI

## 待办

1. **本地全量链接**：若报缺 `RobotScene.lib` / `BackendVisual.lib`，先单独编对应工程再编 `CloudSimHost` / `Widget` / `CloudSim`
2. **手工验收**：按 ACCEPTANCE 清单在 UI 测「点云匹配」「体素下采样」「导入」取消路径
3. **Catalog 同步**：改 `tools/ai-training/catalog/full_api_catalog.json` 后运行 `python tools/ai-training/catalog/_embed_catalog.py` 再编 Host
4. **运行时 ai_config.json**：若 exe 旁已有旧配置，需手工合并新 domains（`pointcloud.ops` 等）或删除后用 defaults 重生
5. **PointNet 预标注**：仍引导至标注面板（无单一 Host API）；若要 Agent 化需另接 PointNet 插件推理路径

## 已跟进

- `AiDomainRouter`：「点选边/面」优先于标注「点选」，避免误路由

## 配置提示

- Ollama：`http://127.0.0.1:11434/v1`，rules 可零模型命中按钮关键词
- 配准对话框要求文档内 ≥2 个点云/网格对象
