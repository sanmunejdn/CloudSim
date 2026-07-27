# CONSENSUS — 碰撞检测

## 需求与验收

1. `CollisionAlgorithm.dll` 提供 `CollisionWorld`（upsert/setPose/exclude/checkAll）
2. 几何：`MeshBackendData::triangleSoup`；`BrepBackendData` 经 `discretizeShapeToMesh`
3. 仿真 Dock「启用碰撞检测」默认关；安全余量默认 1 mm
4. `enabled==false` 时 plan/Run 不调用碰撞
5. `enabled==true` 时轨迹抽样命中 → `PlanResult.ok=false` + 摘要
6. 相邻连杆不误报；工程可读写 `robotCollision`

## 技术方案

| 层 | 职责 |
|----|------|
| CollisionAlgorithm | soup→BVH/AABB、位姿、ACM、checkAll |
| RobotScene | `RobotCollisionSettings`、轨迹抽样校验 |
| Host/Controller | Backend 同步、FK 后更新 pose、plan 挂钩 |
| RobotWidget | Dock 开关 UI |

单位：**mm**；位姿契约见 `docs/spatial_contract_world_pose.md`。

## 约束

- 新代码不直链 OSG；CollisionAlgorithm 不依赖 Data 类型（只收 soup+Mat4）
- 无第三方 coal 时内置实现须可编译可运行
