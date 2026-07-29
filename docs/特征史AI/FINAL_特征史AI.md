# FINAL — feature.compose Parametric 特征史

AI ActionPlan 已接到 Host `*ToBrep` API，真正写入 `ParametricBrepBackendData` 特征链。

## 交付

- Domain：`feature.compose`（Handler + 路由 + LLM prompt + 规则 Pad）
- 步骤：`extrudeSketchProfileToBrep`、`filletEdgesToBrep`（含 `edges=all`）、`linearPatternBodyToBrep`、`askClarify`
- ABI **1.43.0**：`onParametricBodyHistoryChanged`；Fillet `allEdges`
- 几何建模插件订阅回调 → `syncFeaturesFromBody`

## 文档

`CONSENSUS` / `DESIGN` / `TASK` / `ACCEPTANCE` 于本目录；思想说明见 `../硬化基准面/TEXT2CAD_AI助手.md`（可对照更新）。
