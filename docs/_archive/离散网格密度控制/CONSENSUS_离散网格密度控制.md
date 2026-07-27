# 离散网格密度控制 - CONSENSUS

## 需求

Geometry 离散支持互斥三模式：质量预设 / 目标边长(mm) / 目标三角面数。

## 技术方案

| 层级 | 行为 |
|------|------|
| `geoalgo::MeshDiscretizeParams` | `densityControl` + `targetEdgeLengthMm` + `targetTriangleCount` |
| 边长 | OCC `deflection≈target×0.25`，`angular=1°`；`refine` 预加密到 `1.5×target`（上限 40 万面） |
| 面数 | 相对 deflection 二分，约 8 次，容差约 ±15% |
| Data `discretizeStepToMesh` | 边长模式：repair → 单次 `isotropicRemesh`(3 iter)；失败保留 refine；tris>80 万才跳过 remesh |

## 明确不做（已否决）

1. OCC 基网格刻意偏粗（少先打成过密圆角）
2. `isotropicRemeshProgressive`（由粗到细）
3. remesh 失败后再最长边细分兜底（细分只在 algo、remesh 之前）

## 验收标准

1. 三模式 UI 互斥且参数透传到 Data/algo
2. 面数模式：面数相对目标偏差通常在约 2× 内（SelfTest 门槛）
3. 边长模式：平面区平均边长接近目标；失败不丢网格
4. 不使用上述三项否决路径
