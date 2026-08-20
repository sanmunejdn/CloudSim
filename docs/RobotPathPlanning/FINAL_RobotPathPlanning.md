# FINAL — RobotPathPlanning

## 交付摘要

新建 `RobotPathPlanning.dll`，实现 MoveIt 风格关节空间避障规划：TCP 位姿目标经 IK 转为关节目标，**BIT***（路径长度 anytime 最优）为主、InformedRRT*/RRT*/RRTConnect 级联兜底，在 `CollisionWorld` 上规划，输出 **关节轨迹 + TCP 位姿点列**。仿真 Dock「碰撞与规划」提供起终点路点选择、预览与确认插入 Pmid。

## 工程与产物

| 项 | 路径 |
|----|------|
| DLL | `CloudSim/src/Robot/RobotPathPlanning/` |
| Debug | `bin/x64d/RobotPathPlanning.dll` |
| Release | `bin/x64/RobotPathPlanning.dll` |
| 公共头 | `inc/RobotPathPlanning.h` |
| 文档 | `CloudSim/docs/RobotPathPlanning/`、`DEVELOPER_GUIDE.md` |

已加入 `CloudSim.sln`、`CloudSimWeb.sln`；`RobotWidget` 链入并暴露 Dock 入口。

## 技术要点

- **规划器**：`OmplJointSpacePlanner`（需 `CLOUDSIM_HAS_OMPL`）；默认 `BITstar` + `PathLengthOptimizationObjective`；无 OMPL 时内置 RRT。
- **碰撞位姿**：`M = m0 * inv(T0) * Tq * P`，绑定矩阵以 `osg::Matrixd` 传入（勿跨 DLL `std::function` apply）；densify 后复检；UI `validateJointTrajectory` 画面闸门。
- **黑白名单**：同名单不互碰，仅跨名单对；设置持久化 `whiteListBackendIds` / `blackListBackendIds`。
- **插入**：`insertRawTrajectoryBetween` + 分组成员，Pmid 落在起终点之间；示教关节 CSV 写回。
- **运动学**：`RobotUrdf` FK/IK + `GeometryEngine::ToolKinematics`。
- **不引入** MoveIt2/ROS；不写程序指令；不保证笛卡尔直线。

## 相关缺陷修复（同批）

- 规划/仿真位姿不一致导致「规划通过、回放碰撞」。
- 打开工程时 IO 信号页 UAF：`QPointer` + `fromProjectJson` 期间 `QSignalBlocker`。

## 编译验证

- RobotPathPlanning / RobotWidget / 依赖链：Debug|x64 与 Release|x64 均须通过（见仓库 VS 生成规则）。
