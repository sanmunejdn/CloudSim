# ALIGNMENT — 跟随对象框架隔离

## 原始需求

填充跟随对象后机器人解体；需分析原因，从框架层修改，实现工件/工具相对机器人末端的恒定相对跟随，且机器人保持装配完整。

## 项目理解

- 机器人连杆世界位姿由 FK（`RobotSceneKinematics` / per-link）写入 OSG。
- 自由物体跟随由 `FollowAttachment` + `BackendFollowTransformSolver` 写 `pose` 再 `syncOuterPatFromBackend`。
- 工程 `edges[]` 会经 `applyHierarchyFollowBinding` 给**子节点**装 `hierarchyDriven` Follow；URDF 导入本身不装 Follow，但工程加载会装回连杆。

## 边界

| 在范围内 | 不在范围内 |
|----------|------------|
| Follow / FK 位姿所有权隔离 | 改 URDF 导入拓扑 |
| 工程 edges / 属性绑定路径 | 轨迹算子逻辑 |
| 旧工程误装 Follow 的迁移卸除 | 矩阵打包旧 bug 回退（已修，保持） |

## 根因（已确认）

1. 工程 edges 给机器人连杆装 Follow follower。
2. 工件绑定触发 `requestFollowSolveForced()` → 全量求解写**所有** follower（含连杆）`pose`，再按 `listData()` 非拓扑序 sync，与 FK 抢写 → 解体。

## 疑问（已自决）

- 机器人是否允许手工 Follow？**否** — `sourceType=URDF` 一律 FK 独占。
