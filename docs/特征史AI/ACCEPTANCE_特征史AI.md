# ACCEPTANCE — feature.compose → Parametric 特征史（含续期）

| 项 | 状态 |
|----|------|
| Domain `feature.compose` 注册 | 完成 |
| Router：建模+孔 → feature；纯布尔 → mesh | 完成 |
| LLM `featureComposeSystemPrompt`（Pocket/Chamfer/Revolve） | 完成 |
| `extrudeSketchProfileToBrep` 写 `sketchDocumentJson` | 完成 |
| Pad + Pocket 通孔规则解析 | 完成 |
| Fillet / Chamfer（`edges=all`）/ LinearPattern / Revolve | 完成 |
| `askClarify` | 完成 |
| `onParametricBodyHistoryChanged` + geomodeling sync | 完成 |
| ABI 1.44.0 `0x00012C00`（Chamfer `allEdges`） | 完成 |
| Debug\|x64 编译 | 通过 |

## 手测

1. AI：「建模 100x80x40」→ Parametric Body；双击 Sketch → 可改边长并 rebuild  
2. 「建模 100x80x40 中心通孔 d10」→ history 含 Sketch+Pad+Sketch+Pocket；**无** MeshBackendData  
3. 「倒角 / 旋转圆柱 半径50 高100」→ Chamfer / Revolve 写入 tip  
4. 纯「布尔差集挖孔」仍可走 mesh.compose  
5. 模糊尺寸 → askClarify
