# ALIGNMENT — 草图硬化（命名参数 + 包 A）

## 项目上下文

CloudSim 几何建模插件（`com.cloudsim.geomodeling`）已具备 PlaneGCS 草图与 Pad/Sweep 等实体特征；活跃文档见 `docs/几何建模/`。参考 [Yi3D](https://github.com/wangyao1052/Yi3D) 的命名参数与样条双模式语义，不移植其 Transaction/Element 架构。

## 原始需求

落地官方优先栈本期部分：

1. 命名参数面 MVP（侧栏改参 → Solve/Rebuild）
2. 椭圆进 GCS + 长短轴参数语义
3. Convert 保圆/弧类型
4. 样条双模式（过点 / 控制点）

并在 ROADMAP 锁定下期 B/C 与 TopoNaming 专题。

## 边界

| 做 | 不做 |
|----|------|
| A0–A3 + 文档锁定后续 | 包 B/C/D 功能代码 |
| 插件内参数表 | Yi3D DataBase/事务 |
| 圆/线/椭圆 + Pad 深度参数 MVP | 全特征参数一次铺满 |
| 样条控制多边形 | 整段 BSpline GCS |

## 需求理解

- 侧栏复用 `GeometricModelingPage` 的 `m_toolStack`，增加参数页
- 椭圆进 `SketchConstraintSolver`；Convert 硬化 `ShapeQuery`
- 样条 JSON 兼容旧 `throughPts`

## 疑问澄清

- 属性框完整 SW 化 → 否，本期 MVP
- 插件链 OCC 做样条拟合 → 否，无 OCC 依赖；控制点用简化 poles 生成
