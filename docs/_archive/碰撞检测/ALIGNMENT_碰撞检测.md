# ALIGNMENT — 碰撞检测

## 原始需求

在 CloudSim 内集成碰撞检测：几何取自文档后端已显示的 Mesh/B-rep（不依赖 URDF `<collision>`），规划阶段抽样校验；仿真 Dock 提供总开关（默认关）。

## 项目上下文

- 技术栈：Qt + OSG + OCC + CGAL + Eigen；机器人子系统 RobotScene / RobotUrdf / RobotWidget
- 连杆几何：URDF 导入后每 link 一个 `MeshBackendData`（visual）
- 规划入口：`RobotInstruction::Controller::plan` → `RobotSimulationController::planMotionOnHost`
- 本机无预置 `bin/SDK/coal` 时：模块内置 AABB+三角窄相位；存在 coal 时可用 `CLOUDSIM_HAS_COAL`

## 边界确认

| 纳入 | 不纳入（一期） |
|------|----------------|
| Mesh/B-rep（离散）后端几何 | URDF `<collision>` 解析 |
| 自碰 + 场景环境体 | 连续碰撞 CCD |
| plan 抽样 + 总开关/安全余量 | 动力学仿真、MoveIt/Pinocchio |
| 工程持久化 `robotCollision` | B-rep `BRepExtrema` 精确窄相位 |

## 需求理解

1. 碰撞真源 = 视口同源后端几何
2. 总开关默认关，关闭时零开销短路
3. ACM 邻接排除来自 URDF joint 父子，不依赖 collision mesh

## 疑问澄清（已决策）

| 问题 | 决策 |
|------|------|
| 用 URDF collision 还是后端？ | 后端 Mesh/B-rep |
| 无 coal 怎么办？ | 内置 mesh CD；coal 可选加速 |
| 隐藏对象是否参与？ | 以几何存在为准，与可见性无关 |
| 设置挂文档还是全局？ | 文档级，随 `.pcp` |
