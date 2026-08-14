# CONSENSUS — AI 助手：计划可校验 + 执行可观测 + 确认 UX 单轨

## 验收标准

1. Catalog 工具在确认提交后、执行前按 `args_schema` 校验必填参；缺参则失败并提示（确认面板可先补参）。
2. `domain=="mesh.compose"` 的 ActionPlan 在执行前走 `MeshComposeDomainHandler::validatePlanJson`（与 feature.compose 对称）。
3. Agent 单步执行成功后，观测串含 `new=...`（当场景新增 backend 时）；Memory/LLM 观测回灌可用。
4. 「确认 / 确认并离散」走 `beginDomainConfirm(TrajectoryCommit)` → 弹现有离散对话框 → Accept 后 Runtime `commitAiTrajectoryFeatures`；「返回重选」发 Secondary 且**保留**特征会话；取消保留选择。
5. `CloudSimHost` Debug|x64 与 Release|x64 编译通过。

## 技术方案摘要

见 `DESIGN_AI助手计划校验观测确认单轨.md`。
