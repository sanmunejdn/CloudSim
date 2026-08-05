# CONSENSUS — 草图硬化（命名参数 + 包 A）

## 需求描述

在 GeometricModelingPlugin 内完成 A0–A3，并文档锁定后续优先栈第 3–4 项。

## 验收标准

1. **A0**：选中圆/线/椭圆可在侧栏改半径或长短轴/端点相关参数 → Solve；选中 Pad/Pocket 可改深度 → Rebuild；存盘可恢复
2. **A1**：椭圆参与 PlaneGCS；长短轴尺寸/约束可驱动几何
3. **A2**：圆柱端面 Convert 优先出 Circle/Arc；折线边有警告统计
4. **A3**：样条可进入控制点模式、拖 poles、JSON 兼容旧文档
5. Debug|x64 与 Release|x64：GeometryAlgorithm（若改）+ GeometricModelingPlugin 通过

## 技术方案

- 参数：插件内字符串键（对齐 Yi3D 命名习惯），无 FieldId
- 求解：扩展 `SketchConstraintSolver` + `syncConstraintsToSolver`
- Convert：`extractShapeFaceBoundarySegments` mid/满圆硬化 + Convert 日志
- 样条：`SkSplineMode` + `controlPts`；无 OCC，过点模式生成简化控制多边形

## 任务边界

不做：B/C/D 代码、TopoNaming 引擎、整段 BSpline GCS、椭圆弧实体。

## 参考

Yi3D `SketchParamNames` / `SplineMode` / `SketchEllipse` 语义；CloudSim 保留 PlaneGCS。
