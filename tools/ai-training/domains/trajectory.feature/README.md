# trajectory.feature 专模

**场景：** 轨迹生成页已选 STEP 工件 + 用户文本 + catalog 切片 JSON → 线/面意图与候选编号 → 确认后离散轨迹。

**运行时开发文档（会话、3D 叠加、API）：** [`../../../docs/trajectory_feature_ai.md`](../../../docs/trajectory_feature_ai.md)

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5:3b`（文本 + catalog JSON） |
| `multimodal` | `false` |
| `parser_priority` | `["rules", "local"]`（与 `ai_config.defaults.json` 一致；可追加 `remote`） |
| 训练集 | `dataset.jsonl`（`gen_trajectory_feature_dataset.py` 可从 STEP 批量生成） |

## 输出 schema（version 1）

```json
{
  "version": 1,
  "featureAxis": "line|surface|ambiguous",
  "clarifyMessage": "ambiguous 时必填",
  "selectedCandidateIds": ["edge_3"],
  "features": [],
  "suggestedPipelineTemplate": "weld_default|glue_default|grind_default"
}
```

LLM / rules 输出 `selectedCandidateIds`；`features[]` 可在确认前为空，由宿主 `buildFeaturePlanFromCandidateIds` 或 Coordinator 本地补全。

## catalogSlice 格式

每条候选含 `displayIndex`（1..N）、`candidateId`、`suggestedKind`、`summary`、`refs`。

切片字段 `featureAxis`：`line` / `surface` / `ambiguous`。

## 交互与高亮

| 阶段 | 3D 视口 |
|------|---------|
| 首次识别成功 | 切片内**全部**候选编号高亮 |
| 用户「选 1 和 3」 | **仅** displayIndex 1、3 对应特征高亮（`filterCatalogSliceByCandidateIds`） |
| 确认并离散 | 清除特征叠加，显示 raw 轨迹预览 |

## 运行时验收

1. 轨迹页选 STEP → AI「轨迹特征」→「识别焊缝边」→ 列表编号 + 3D 全量高亮。
2. 「识别打磨面」→ 仅面候选；「线特征」→ 切换线候选。
3. 「识别特征」→ 线/面澄清，不误走 `geometry.recognize`。
4. 「选 1 和 3」→ 3D 仅 #1、#3；对话显示「已选中特征」。
5. 「确认并离散」→ 离散 + 默认工艺流水线（weld/glue/grind）。
6. catalog 为空时 Coordinator 自动 rules 重试一次。
7. `python scripts/build_dataset.py trajectory.feature` 通过。
