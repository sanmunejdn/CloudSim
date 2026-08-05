# ACCEPTANCE — 硬化 + 基准面入门（含续期）

| 任务 | 状态 | 说明 |
|------|------|------|
| T0 ABI 1.40.0 | 完成 | `0x00012800`；hitWorldMm；face 边折线 API |
| T1 Convert | 完成 | `discretizeBackendFaceEdgesToPolylines` |
| T2 UpToVertex | 完成 | Vertex 拾取吸附近端点 |
| T3 Draft 中性面 | 完成 | 侧栏点选中性面 |
| T4 Offset | 完成 | 外环+孔环；自交拒绝 |
| T5 多边形 | 完成 | 3–24 边对话框 |
| T6 DatumPlane | 完成 | 等距面/三点；树双击开草图；sync 保留 |
| T7 DatumPlane 持久化 | 完成 | `onProjectAboutToSave` / `onProjectLoaded` 写入/合并 `geometricModeling.features` |
| T8 基准面视口 overlay | 完成 | `appendVisibleSketchOverlays` 绘制约 80×80 mm 青绿边框 |
| T9 特征级线性阵列 | 完成 | `patternSourceFeatureId`；侧栏「整个实体 / 上游特征」；ABI **1.41.0** |
| T10 Text-to-CAD → AI | 完成 | `feature.compose` + P1 智能选边；见 `../特征史AI/` |
| Convert 圆弧 | 完成 | `faceBoundarySegs` → Arc/Circle |
| 成角基准面 | 完成 | `DatumPlaneAngle` |
| 真 Vertex | 完成 | TopExp + UpToVertex 索引 |

## 编译验收

- [x] GeometryAlgorithm / Data
- [x] CloudSimPluginSDK（续期 ABI 见 `../特征史AI/`，当前 **1.44.0**）
- [x] CloudSimHost（含 PluginGeometryHostImpl、AiLlmClient）
- [x] GeometricModelingPlugin（Debug|x64）

## 手测建议

1. 建 DatumPlane → 保存工程 → 重开：树节点与视口框仍在  
2. 阵列侧栏选「上游特征」→ 预览/提交与「整个实体」行为不同  
3. AI：「建模 100x80x40 中心通孔 d10」→ Pad+Pocket 特征链（非 mesh）  
4. AI 模糊尺寸 → `askClarify`
