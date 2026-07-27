# ALIGNMENT：三点圆弧指令（CIRC / ARC）

## 原始需求

在 CloudSim 机器人仿真中增加圆弧运动指令：通过三点定圆（工业 CIRC 语义），设计指令 UI、后端数据模型与运动学求解（笛卡尔弧采样 + IK）。

## 项目上下文

- 现有运动路点：`Type::PTP` / `Type::LINE`（[`RobotInstructionModel.h`](../../src/Robot/RobotScene/inc/RobotInstructionModel.h)）
- 规划：`PtpPlanner` / `LinePlanner`（[`RobotInstructionController.cpp`](../../src/Robot/RobotScene/source/RobotInstructionController.cpp)）
- UI 添加：`SimulationCommandWidget` → `RobotSimulationController::onSimulationAddInstructionRequested`
- 预览/Run：链式种子 + `jointTrajectoryRad`；`isMotionWaypointType` = PTP|LINE
- 运动学库：`RobotKinematics` 仅 FK/位置 IK；弧几何应新增纯几何 API

## 边界确认

| 纳入 | 不纳入（本期） |
|------|----------------|
| `Type::ARC` + Via/End 存储 + JSON | 品牌 MoveC / FANUC C 导出脚本 |
| 两步示教 UI | LINE corner blend「圆弧过渡」 |
| ArcPlanner + 预览/Run 贯通 | Via 姿态强制约束、多圈弧 |
| RobotKinematics 三点定圆采样 | 独立 MotionPlanSession 下沉 |

## 需求理解

1. **语义 1A**：起点 = 上一段运动终点（规划时种子关节 FK）；指令存 Via + End
2. **两步示教**：点 ARC 捕获 Via → 再点确认 End → 落盘一条指令
3. **姿态**：位置走圆弧；姿态 Slerp(Start→End)；Via 姿态持久化供面板
4. **与 LINE 对齐**：采样密度、lite 升采样、PlanResult / Executor 轨迹插帧

## 疑问澄清（已确认）

| 问题 | 结论 |
|------|------|
| 三点如何定义 | 1A：Start 隐式 + Via/End 在指令上 |
| 示教交互 | 两步示教 |
| 品牌导出 | 本期不做 |
