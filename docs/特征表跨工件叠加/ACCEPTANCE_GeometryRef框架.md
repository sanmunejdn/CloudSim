# GeometryRef 立刻替换 — 验收

BugTracker: **#93**（fixed，待桌面点验）

## 已完成

- `FeatureGeometry` 仅 `refs`；写 JSON 无 `edgeIndices`/`faceIndices`；读路径升级旧键
- `getFeatureGeometrySchema` / StrategyCatalog `geometrySchema`
- `FeatureDiscretizeContext` + `discretizeEx`；`FaceOffsetCurve` 跨工件（预变换折线）
- `buildFeatureDiscretizeContext` + `discretizeFeatureListAcrossWorkpieces` 逐行编排
- 桌面/Web 按 schema 闸门；`FaceOffsetCurve` 允许跨工件追加；primary=`targetFace`
- Debug|x64 / Release|x64：GeometryAlgorithm → … → CloudSimHost；Web build:debug/release

## 点验

1. 同工件 EdgeChain / FaceOffsetCurve 行为与改前一致
2. FaceOffsetCurve：边@零件A、面@零件B → 轨迹贴合 B，表工件列为 B
3. EdgeChain 异工件追加仍被拒
