# FINAL — 非刚性配准轨迹算子

## 摘要

新增流水线算子 **NonRigidRegistration**：在 scope 内将轨迹点绑定到源点云/mesh，对源几何拷贝同步调用已有 SPARE，按绑定写回 `poseMm`。源/目标均支持点云或 mesh 四种组合。

## 关键文件

| 文件 | 说明 |
|------|------|
| `TrajectoryGeometryResolver.cpp` | 绑定 + SPARE + 写回（与几何投影同文件） |
| `ops/NonRigidRegistration/*` | 原子块四件套 |
| `INonRigidTrajectoryWarp.h` | ctx 注入接口 |

## 设计要点

- Builtins 不链 Data.lib；SPARE 经 RobotScene 适配
- 不写回场景 backend；仅改轨迹位姿
