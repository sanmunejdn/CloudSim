# CONSENSUS — 实体扫描特征

## 验收标准

1. 路径草图（线/弧）+ 闭合轮廓 → 扫描凸台可见
2. 扫描切除切穿已有 Pad
3. 改路径/轮廓草图后下游 Sweep 重建
4. Undo + history JSON 含 `Sweep`/`SweepCut` + `pathSketchRefId`
5. Host 1.31.0+；Algo→Data→SDK→Host→插件 Debug 编译通过

## 技术方案

| 层 | 方案 |
|----|------|
| Algo | `geoalgo::sketchSweepPolylineToHandle` |
| Data | kind Sweep/SweepCut；profile+path xyz；rebuild |
| Host | preview/commit Sweep API |
| 插件 | Ribbon + 双草图下拉侧栏 |
