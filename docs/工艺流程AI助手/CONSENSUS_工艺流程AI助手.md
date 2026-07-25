# CONSENSUS — 工艺流程 AI 助手

## 需求与验收

1. 进入工艺流程后可见 AI 助手并可发消息；返回三维后侧栏恢复
2. 「生成 N 工位流水线并仿真」→ 确认 → 画布合法图 → 聊天含 Makespan/吞吐等
3. 非法图工具失败且不破坏已有合法图（先校验再 `fromJson`）
4. auto 域工艺/产线口语路由到 `process.flow`
5. Host &lt; 1.20.0 时插件拒绝加载

## 技术方案

- Host：`setProcessFlowAiBridge` / `processFlowAiBridge`
- 插件：`ProcessFlowAiBridge` → `fromJson` + `autoLayout` + `DesEngine::run`（同步）
- 域：`process.flow`；Catalog：`applyProcessFlowGraph` / `runProcessFlowSimulation` / `compareProcessFlowPolicies`
- 规则：`AiProcessFlowRules` 线性产线模板

## 约束

- AI 不直接链接 ProcessFlow 私有类
- 首版整图替换，非增量编辑
