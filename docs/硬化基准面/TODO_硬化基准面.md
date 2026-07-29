# TODO — 硬化 + 基准面入门

## 部署

1. 宿主与插件须同为 **1.41.0+**（`0x00012900`）。
2. 重新编译并部署：`CloudSimHost.dll` + `GeometricModelingPlugin.dll` + `GeometryAlgorithm.dll` + `CloudSimPluginSDK.dll`（及依赖 Data）。

## 已知债

| 项 | 说明 |
|----|------|
| Vertex | 边端点吸附，非 TopExp 顶点索引 |
| Offset | 孔环统一反号；复杂尖角仍可能失败 |
| 成角基准面 | 未做 |
| Pattern 语义 | 选上游 tip 后，源特征与 Pattern 之间的中间特征可能被 tip 替换抹掉 |
| AI 几何执行 | 仍走网格代理；尚未映射到 `extrudeSketchProfileToBrep` 等 Parametric Host API |

## 可选下一步

- AI ActionPlan → Parametric Host（真正 text-to-CAD 特征链）
- Convert 保留圆弧（FaceBoundaryProjector）
- 成角基准面 / DatumAxis
