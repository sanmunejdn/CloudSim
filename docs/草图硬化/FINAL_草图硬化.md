# FINAL — 草图硬化

按 CONSENSUS 完成 A0–A3：命名参数面 MVP、椭圆 PlaneGCS、Convert 保圆/弧、样条双模式；借鉴 Yi3D 参数命名与 SplineMode 语义，未引入其事务架构。

## 主要改动

- Plugin：`GeometricModelingPage` Params 栈；`SketchEditSession` 选中/命名参数；`SketchConstraintSolver` 椭圆；`SketchGeom` 样条 mode/controlPts  
- GeometryAlgorithm：`ShapeQuery` 满圆/周期 mid  
- 文档：`docs/草图硬化/` + 更新 `docs/几何建模/FEATURES`/`ROADMAP`

## 编译

Debug|x64 与 Release|x64：GeometryAlgorithm、GeometricModelingPlugin 通过。无 Host ABI bump。
