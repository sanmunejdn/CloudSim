# ALIGNMENT — AI 助手：计划可校验 + 执行可观测 + 确认 UX 单轨

## 原始需求

落实「计划可校验 + 执行结果可观测 + 确认 UX 单轨」。

## 边界

| 做 | 不做 |
|----|------|
| Agent 提案后按 Catalog `args_schema` 校验必填参 | 关掉 `require_keyword_hit` / 全自主 Agent |
| `AiActionPlanExecutor` 对 `mesh.compose` 与 `feature.compose` 对称校验 | 把离散算子表塞进 `AiConfirmPanel` |
| 执行后用场景快照差分填充 `AiToolResult.newBackendIds` | 重写 Agent FSM |
| 轨迹确认会话并入 `AiAgentRuntime::TrajectoryCommit`，仍弹 `TrajectoryPlanConfirmDialog` | 修改「修改离散参数」修订路径产品语义 |

## 关键决策（已定）

轨迹离散 UI 保持模态对话框（与 `docs/轨迹离散AI确认对话框` 一致）；**单轨**指确认会话进入 Runtime，NeedConfirm 时 Coordinator 弹对话框，确认后 `submitAgentConfirm(merged)` 由 Runtime 提交。
