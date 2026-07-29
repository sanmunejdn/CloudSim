# TODO — feature.compose

## 部署

宿主 + 插件 + GeometryAlgorithm 同编 **1.43.0+**。

## 已知债

| 项 | 说明 |
|----|------|
| Sweep/Loft/Shell/Draft/Revolve | 未进 ActionPlan |
| Fillet 拓扑 | `edges=all` 可能过猛；智能选边未做 |
| 草图可编辑性 | AI Pad 未写完整 `sketchDocumentJson` |
| mesh vs feature 混合句 | 「拉伸再挖孔」现优先 feature；孔需 Pocket 或仍靠 mesh |

## 可选下一步

- Pocket 矩形孔规则解析
- ActionPlan → Chamfer / Revolve
- AI 写入可编辑草图 JSON
