# TODO — feature.compose 续期

## 部署

宿主 + 插件 + GeometryAlgorithm 同编 **1.44.0+**（`0x00012C00`）。

## 已知债

| 项 | 说明 |
|----|------|
| Sweep/Loft/Shell/Draft | 未进 ActionPlan |
| Fillet/Chamfer 拓扑 | `edges=all` 可能过猛；智能选边未做 |
| Revolve 轴 | AI MVP 固定原点+Y；UI 可选轴未完全透出 |
| Pattern 语义 | 上游 tip 中间特征可能被抹掉 |

## 可选下一步（P1）

- Fillet 智能选边（最长边 / 顶面边界）
- Convert 保留圆弧
- 成角基准面
- 真 Vertex 拾取
