# ACCEPTANCE — 草图硬化

| 项 | 状态 |
|----|------|
| A0 命名参数面：圆/线/椭圆侧栏改参 → Solve | 完成 |
| A0 Pad/Pocket 树选改 depth/draft → Rebuild | 完成 |
| A1 椭圆进 PlaneGCS + Major/MinorRadius | 完成 |
| A1 半径尺寸可打椭圆长半轴 | 完成 |
| A2 ShapeQuery 满圆容差 + 周期 mid | 完成 |
| A2 Convert 分类日志 + 折线警告 | 完成 |
| A3 SkSplineMode + controlPts JSON 兼容 | 完成 |
| A3 控制点模式 overlay 虚线多边形 | 完成 |
| Debug\|x64 GeometryAlgorithm + Plugin | 通过 |
| Release\|x64 GeometryAlgorithm + Plugin | 通过 |
| ROADMAP 锁定 B/C 与 TopoNaming | 完成 |

## 手工建议

1. 草图点选圆 → 侧栏改 radius → 求解变形  
2. 椭圆加半径尺寸 → Solve  
3. Convert 圆柱端面 → 日志含 circle/arc 计数  
4. 样条参数 mode=1 → 拖控制点  
5. 特征树点 Pad → 改 pad.depth → Rebuild  
