# ALIGNMENT：机器人指令执行 / 预览 / 运行

## 原始需求

理清机器人指令执行、预览、运行是一次性全运算还是动态运算；分析可升级点；修复「末尾点规划失败导致整段不播」。

## 项目理解

- `RobotScene`：指令模型、Planner、`RobotProgramExecutor`、场景 FK
- `RobotKinematics`：DH 回退 IK
- `RobotWidget`：`RobotSimulationController` 编排预览/Run/缓存

## 边界

- 本轮实现：部分失败停机策略、Executor 轨迹优先、LINE 笛卡尔采样、文档同步
- 不在本轮：MotionPlanSession 下沉、梯形速度剖面、跳过失败点

## 已确认决策

- 失败策略：播到失败点前停止
- 播放起点：程序起点关节（`playbackStartAngles`），不被链式 rollingQ 覆盖
