# FINAL — 硬化 + 基准面入门（含续期）

按 CONSENSUS 完成 Convert 真面边界、UpToVertex、Draft 中性面、Offset 孔环+自交、多边形 3–24、用户基准面；并完成续期三项 + Text-to-CAD 思想落地。

宿主 ABI：**1.41.0**（`0x00012900`）。插件门控同版本。

## 续期交付

| 项 | 要点 |
|----|------|
| DatumPlane 持久化 | 工程 JSON `geometricModeling`；加载后 `appendPreserved` |
| 视口 overlay | 可见基准面青绿矩形边框 |
| 特征级 LinearPattern | `sourceFeatureIdUtf8` / 侧栏源特征；重建用 `tipAfterFeature` |
| AI text-to-CAD | 顺序特征 prompt、`askClarify`、路由关键词；几何仍为 mesh 代理 |

## 主要改动面

- Host/SDK：ABI、`PluginSketchLinearPatternParams::sourceFeatureIdUtf8`
- Plugin：save/load、overlay、Pattern UI、FeatureDocument JSON
- Parametric：`patternSourceFeatureId`、`tipAfterFeature`
- AI：`AiLlmClient` / `AiActionPlanExecutor` / `AiDomainRouter`
- 文档：`TEXT2CAD_AI助手.md`
