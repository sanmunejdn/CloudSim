# FINAL — feature.compose Parametric 特征史（续期包）

AI ActionPlan 写入可编辑草图 JSON；板+通孔走 Pocket 特征链；Chamfer/Revolve 已挂执行器。

## 交付

- **F1**：`extrude` / `revolve` 前生成 `SketchDocument2d` 兼容 JSON → `sketchDocumentJsonUtf8`
- **F2**：Router（建模+孔 → feature）；规则 Pad+Pocket；Prompt 禁止 booleanMesh 通孔
- **F3**：`chamferEdgesToBrep` / `revolveSketchProfileToBrep` + 白名单 + Prompt；Chamfer `allEdges`
- ABI **1.44.0**：Chamfer `allEdges`（Fillet `allEdges` / history 回调仍为 1.43 起）

## 文档

本目录 `TODO` / `ACCEPTANCE` / `FINAL`；硬化基准面 TODO 已去掉过时「AI 仍网格」条目。
