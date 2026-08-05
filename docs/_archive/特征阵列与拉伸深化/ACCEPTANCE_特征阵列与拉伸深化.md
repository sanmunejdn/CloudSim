# ACCEPTANCE — 特征阵列与拉伸深化

## 验收对照

| ID | 验收项 | 结果 |
|----|--------|------|
| T1 | `tipBeforeFeature` + `featureContributionSeed`；线性/圆周 Pattern 源特征 = after CUT before；Host preview 同步 | 代码已落地 |
| T2 | 成角基准面铰链方向取边 `edgeEndA/B`，失败回退 `axisX` | 代码已落地 |
| T3 | FEATURES/ROADMAP 标圆周与 tip/成角已交付；与 T1 共用 seed | 文档已对齐 |
| T4 | `startOffsetMm` + `TwoDirections`/`length2Mm`；JSON 兼容；Host ABI **1.48.0**；侧栏 UI | 代码已落地 |
| Topo | `docs/TopoNaming/ALIGNMENT_TopoNaming.md` 锁定范围，无命名引擎代码 | 已锁定 |
| Build | GeometryAlgorithm → Data → SDK → CloudSimHost → Plugin，Debug\|x64 + Release\|x64 | **通过** |

## 手工点验建议（编译通过后）

1. Pad → Fillet → LinearPattern(源=Pad)：只阵列凸台贡献，Fillet 保留在 tip
2. 成角基准面：选面+边+30°，旋转轴与边平行
3. CircularPattern：轴边 + count/angle + 特征源
4. Blind + startOffset；双向 10/5；存盘重开
