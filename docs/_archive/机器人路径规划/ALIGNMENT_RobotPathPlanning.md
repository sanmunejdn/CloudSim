# ALIGNMENT — RobotPathPlanning

## 原始需求

关节空间避障路径规划：给定起点、TCP 位姿终点、场景 mesh/STEP 障碍；输出关节轨迹 + TCP 位姿点列；Dock 预览。

## 边界

| 做 | 不做 |
|----|------|
| 新 DLL `RobotPathPlanning`；OMPL 关节空间；CollisionWorld mesh 碰撞 | MoveIt2 / ROS |
| 复用 RobotUrdf IK/FK、BackendCollisionSync | PathPlan 流水线 / 写程序 |
| Dock 简单按钮预览 | 笛卡尔直线避障、外轴联立 |

## 单位与契约

- 长度 mm；关节 rad；矩阵列主序；`T_base_target` 经 `flangeFromToolOrigin` 送 IK。
