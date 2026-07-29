# ACCEPTANCE — feature.compose → Parametric 特征史

| 项 | 状态 |
|----|------|
| Domain `feature.compose` 注册 | 完成 |
| Router CAD 词 → feature.compose；布尔 → mesh.compose | 完成 |
| LLM `featureComposeSystemPrompt` | 完成 |
| `extrudeSketchProfileToBrep` / Fillet / LinearPattern 步骤 | 完成 |
| `askClarify`（feature + mesh compose） | 完成 |
| 规则解析 `tryParseFeatureComposeUserText` | 完成 |
| `onParametricBodyHistoryChanged` + geomodeling sync | 完成 |
| ABI 1.43.0 `0x00012B00` | 完成 |
| Debug\|x64 编译 | 通过 |

## 手测

1. AI：「建模 100x80x40」→ 场景出现 Parametric Body，history 含 Sketch+Pad  
2. 打开几何建模工作区 → 特征树同步  
3. 「挖通孔」仍走 mesh.compose  
4. 模糊尺寸 → askClarify
