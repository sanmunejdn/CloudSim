# 离散网格密度控制 - TODO

## 状态：已恢复（非 progressive 方案）

已回退的坏路径（勿再启用）：

- OCC 基网格刻意偏粗（`deflection≈0.75×edge`、`angular=8°`）
- `isotropicRemeshProgressive`（由粗到细）
- remesh 失败后再用最长边细分做 Data 兜底

当前正确路径见 `CONSENSUS` / `DESIGN`；Host 需在 VS 完整重链后联调。

## 待本地确认

1. VS 中 Rebuild：`GeometryAlgorithm`、`VcgAlgorithms`、`Data`、`GeometryPlugin`、`CloudSimHost`
2. Geometry 面板：质量 / 边长 / 面数三选一后离散
3. 边长模式：平面区边长接近目标；圆角勿被砸坏
4. remesh 失败时仍应落盘（保留 refine 后 soup），状态非假 Done
