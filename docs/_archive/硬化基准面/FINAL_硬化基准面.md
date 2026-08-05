# FINAL — 硬化 + 基准面入门（含续期）

按 CONSENSUS 完成 Convert 真面边界、UpToVertex、Draft 中性面、Offset 孔环+自交、多边形 3–24、用户基准面；并完成续期三项。

本包交付时宿主 ABI 为 **1.41.0**；AI/硬化已升至 **1.46.0**（见 `../特征史AI/`）。

## 续期交付

| 项 | 要点 |
|----|------|
| DatumPlane 持久化 | 工程 JSON `geometricModeling`；加载后 `appendPreserved` |
| 视口 overlay | 可见基准面青绿矩形边框 |
| 特征级 Linear/Circular Pattern | `sourceFeatureId` + tip fuseOnto；圆周阵列轴/次数/角度 |
| AI text-to-CAD | `feature.compose`（含 Sweep/Loft/Shell/Draft/CircularPattern） |

## 主要改动面

- Host/SDK：ABI、`PluginSketchLinearPatternParams::sourceFeatureIdUtf8`
- Plugin：save/load、overlay、Pattern UI、FeatureDocument JSON
- Parametric：`patternSourceFeatureId`、`tipAfterFeature`
- AI：见 `../特征史AI/`
- 文档：`TEXT2CAD_AI助手.md`
