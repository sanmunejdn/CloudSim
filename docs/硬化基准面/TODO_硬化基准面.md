# TODO — 硬化 + 基准面入门

## 部署

1. 宿主与插件须同为 **1.44.0+**（`0x00012C00`）。
2. 重新编译并部署：`CloudSimHost.dll` + `GeometricModelingPlugin.dll` + `GeometryAlgorithm.dll` + `CloudSimPluginSDK.dll`（及依赖 Data）。

## 已知债

| 项 | 说明 |
|----|------|
| Vertex | 边端点吸附，非 TopExp 顶点索引 |
| Offset | 孔环统一反号；复杂尖角仍可能失败 |
| 成角基准面 | 未做 |
| Pattern 语义 | 选上游 tip 后，源特征与 Pattern 之间的中间特征可能被 tip 替换抹掉 |
| Convert 圆弧 | 投影仍可能折线化 |

## 可选下一步

- Fillet 智能选边 / Convert 保留圆弧 / 成角基准面 / 真 Vertex
- 圆周阵列；Pattern tip 语义修正
- AI 扩 Sweep/Loft/Shell/Draft（UI 已有）

> 注：AI → Parametric Host（`feature.compose`）已落地，见 `../特征史AI/`。
