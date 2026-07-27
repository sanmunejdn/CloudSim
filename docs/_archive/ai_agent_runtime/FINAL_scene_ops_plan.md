# FINAL — scene.ops + Agent 规划增强

## 交付

### 阶段 A：scene.ops
- 域 `scene.ops`；Catalog：`removeSceneObject` / `removeAllSceneObjects` / `translateSceneObject` / `rotateSceneObject`
- Dispatch → 删除 / 相对平移·旋转（`PluginDocumentAdapter` 位姿薄封装）
- Router cues；Dock「场景操作」；`AiSceneOpsRules` 多段抽参

### 阶段 B：规划层
- `AiAgentPlan` / `AiAgentPlanBuilder`：规则多段 → 多 keyword 切段 →（多步线索时）`chatPlanJson`
- Runtime 按计划索引执行，同 api 可重复；标题「计划 i/n」；失败 `replan_on_failure` 一次
- `ai_config.agent.enable_plan` / `plan_max_steps` / `replan_on_failure`

## 验收口语（冒烟）

1. 「删除选中对象」→ 高风险确认 → 删除  
2. 「删除全部对象」→ 清空  
3. 「沿 X 移动 10mm」→ 位姿变化  
4. 「绕 Z 旋转 45 度」→ 位姿变化  
5. **「先沿 X 移动 10mm 再沿 Y 移动 5mm」** → 计划摘要 + 两次平移确认  
6. 「体素下采样然后再 ICP」→ 两步计划（keyword 切段或 LLM）  
7. auto 域下删除/移动路由到 `scene.ops`，不进 mesh.create  
8. `enable_plan=false` 时回退单步提案行为  

## 关键文件

- `AiSceneOpsRules.*` / `AiAgentPlanBuilder.*` / `AiAgentRuntime.*` / `AiLlmClient::chatPlanJson`
- `full_api_catalog.json` + embed；`AiDomainIds::sceneOps()`
