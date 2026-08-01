# CONSENSUS — 特征阵列与拉伸深化

## 验收标准

1. Pattern(源=Pad) 在 Fillet 之后：只阵列 Pad 贡献，Fillet 保留在 tip
2. 成角基准面铰链方向与拾取边平行
3. Blind+startOffset 起点偏移；TwoDirections 两向不等长
4. 圆周阵列文档与 tip 语义一致
5. Debug|x64 与 Release|x64：GeometryAlgorithm / Data / SDK / Host / Plugin 通过
6. TopoNaming 仅 ALIGNMENT 锁定，本期无实现代码

## 技术要点

- Data：`tipBeforeFeature` + Cut(after, before) 作 seed；失败回退 tipAfter
- Extrude：`startOffsetMm`、`length2Mm`、`TwoDirections`；ABI 1.48.0
