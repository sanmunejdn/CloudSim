# FINAL — feature.compose + Host 规范收口

P2 功能保留；Host 侧按 ABI/校验规范收口。

## 关键修正

- **虚表**：`previewCircularPattern` / `circularPatternBodyToBrep` 移到 `IPluginGeometryHost` **末尾**；ABI **1.47.0**
- **LLM**：`featureComposeSchema` 走独立 JSON 解析 + `FeatureComposeDomainHandler::validatePlanJson`
- **执行**：`AiActionPlanExecutor` 对 `feature.compose` 先校验再跑步骤
- **DRY**：`resolvePatternSeed` + `clearStagingAndWarn`
- **Loft AI**：`polylineOnPlane` 把 UV 轮廓抬到 `planeA/B`
- **门禁**：`plugin.json` / `initialize` / `ARCHITECTURE` 对齐 1.47.0

## 文档

见本目录 TODO / ACCEPTANCE。
