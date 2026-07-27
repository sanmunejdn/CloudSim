# 离散网格密度控制 - FINAL

## 结论

边长/面数控制已恢复；已否决并移除：偏粗 OCC 基网格、`isotropicRemeshProgressive`、remesh 失败后再最长边细分兜底。

## 路径

1. **Algo**：面数二分；边长用 `deflection≈target×0.25` + `refineTriangleSoupToMaxEdge`。  
2. **Data**：边长模式后单次 `vcgalgo::isotropicRemesh`（repair + SEH）；失败保留 refine 结果。  
3. **插件**：质量 / 边长 / 面数三选一；状态显示 `tris` + `avgEdge`。

## 编译

Debug x64：`GeometryAlgorithm` / `VcgAlgorithms` / `Data` / `GeometryPlugin` 已通过。Host 请本机 VS 完整 Rebuild。
