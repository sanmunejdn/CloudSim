# RobotPathPlanning 模块开发文档

## 1. 模块定位

关节空间避障路径规划（MoveIt/OMPL 风格，无 ROS）：TCP 位姿目标 → IK → **BIT***（路径长度 anytime 最优）→ 路径简化 → densify → 关节轨迹 + TCP 点列。

| 属性 | 说明 |
|------|------|
| x64 输出 | `RobotPathPlanning.dll` |
| 导出 | `ROBOT_PATH_PLANNING_API`（`ROBOT_PATH_PLANNING_LIB`） |
| 单位 | mm / rad |
| 构建 | **Debug\|x64** 与 **Release\|x64** 均须通过（产物分别在仓库根 `bin\x64d\` / `bin\x64\`） |

**非职责**：不写程序指令、不挂 PathPlan 流水线、不保证笛卡尔直线。

## 2. MoveIt / GitHub 对照

| 参考 | CloudSim |
|------|----------|
| [ompl/ompl](https://github.com/ompl/ompl) `demos/OptimalPlanning.cpp`（BIT* / InformedRRT*） | `OmplJointSpacePlanner` |
| [BIT*](https://robotic-esp.com/code/bitstar/) 批量启发树 | 默认 `BITstar` + `PathLengthOptimizationObjective` + `goalRegionCostToGo` |
| [Informed RRT*](https://robotic-esp.com/code/informed-rrtstar/) | cascade 次选（勿再 `setFocusSearch(true)`，会打开构造函数已关闭的 new-state rejection） |
| MoveIt：阈值 0 | 用满时限返回当前最优 |
| OMPL `PathSimplifier::simplifyMax` | `interpolate` 后简化 |

规划顺序：`BITstar` → `InformedRRTstar` → `RRTstar` → `RRTConnect`（连通兜底）→ 内置 RRT。

## 3. 碰撞位姿（与画面一致）

规划采样写 `CollisionWorld` 时须与 `RobotSceneKinematics::applyPerLinkRobotBasePlacement` 同式：

```text
M(q) = m0 * inv(T0) * Tq * P
```

| 量 | 来源 |
|----|------|
| `T0` / `m0` | per-link **绑定**时的 `fkMeshWorldT0` / `outerWorldAtBindByBackendId`（`osg::Matrixd`，**禁止**再经 BackendMat4 往返） |
| `Tq` | `UrdfRobotLoader::computeMeshWorldMatrices` |
| `P` | `robotBasePlacementWorld`（OSG） |
| 起点 | `linkWorldAtStart` 写回 `pose=osg-start`，避免采样残留 |

实现：`CollisionValidity::updateRobotPoses`（`pose=fk-bind`）。densify 后逐点/邻段再检碰；UI 侧 `BackendCollisionSync::validateJointTrajectory`（apply + OSG）作最终画面复验。

**勿**用跨 DLL 的 `std::function` 在规划循环里 `applyJointAnglesRad`（MSVC 边界易失效，且会扭动画面）。

## 4. 公共 API

见 [`inc/RobotPathPlanning.h`](inc/RobotPathPlanning.h)：`planToTcpPose`、`PathResult`（`jointTrajectoryRad` + `tcpPoses`）。

调用方（`RobotSimulationController::runMotionPathPlanFromWaypoints`）须：

1. `BackendCollisionSync::rebuildWorld` + 起点 `applyJointAnglesRad` + `updatePoses`
2. 填 `linkBodies`、绑定 `fkMeshWorldT0` / `outerWorldAtBind` / `robotBasePlacementWorld`（OSG）
3. `planToTcpPose` 成功后再做 OSG 轨迹复验，通过才预览/插入

## 5. 依赖

- `CollisionAlgorithm`、`RobotUrdf`、`GeometryEngine`、`Data`、OSG、可选 OMPL（[`bin/SDK/ompl/README.md`](../../../bin/SDK/ompl/README.md)）

## 6. 相关文档

- [`../../../docs/RobotPathPlanning/`](../../../docs/RobotPathPlanning/)
- [`../RobotUrdf/DEVELOPER_GUIDE.md`](../RobotUrdf/DEVELOPER_GUIDE.md)
- [`../../UI/RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md)（碰撞页 UI、黑白名单、确认插入）
