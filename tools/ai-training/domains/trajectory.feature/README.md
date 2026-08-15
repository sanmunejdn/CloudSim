# trajectory.feature 专模

**场景：** 轨迹生成页已选 STEP 工件 + 用户文本 + catalog 切片 JSON → 线/面意图与候选编号 → **模态对话框确认策略/参数/管线算子** → 离散。

**运行时开发文档：** [`../../../docs/_archive/轨迹离散AI确认对话框/`](../../../docs/_archive/轨迹离散AI确认对话框/) 与 [`../../../docs/_archive/trajectory_feature_ai.md`](../../../docs/_archive/trajectory_feature_ai.md)

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5:3b`（文本 + catalog JSON） |
| `multimodal` | `false` |
| `parser_priority` | `["rules", "local"]` |
| 训练集 | `dataset.jsonl` |

## 输出 schema（version 2）

```json
{
  "version": 2,
  "featureAxis": "line|surface|ambiguous",
  "clarifyMessage": "ambiguous 时必填",
  "selectedCandidateIds": ["edge_3"],
  "features": [{ "featureId": "edge_3", "strategyId": "EdgeChain", "params": { "stepMm": 2.0 } }],
  "suggestedPipelineTemplate": "weld_default|glue_default|grind_default",
  "pipeline": []
}
```

- `strategyId` / `params` / `pipeline` 可省略：宿主 `enrichTrajectoryPlanJsonInPlace` 按策略默认与工艺模板展开。
- 用户在 `TrajectoryPlanConfirmDialog` 中编辑后的 `pipeline[]` 为执行真源。
- 再编辑：口语「修改离散参数 / 改管线算子」→ `loadBoundTrajectoryPlanForAi` → 同对话框 → `reviseAiTrajectoryPlan`。

## 运行时验收

1. 选特征后弹出对话框，可改策略、离散参数、算子列表。
2. 确认后 raw 与管线与对话框一致。
3. 「修改离散参数」可再打开对话框并重离散。
4. `python scripts/build_dataset.py trajectory.feature` 通过。
