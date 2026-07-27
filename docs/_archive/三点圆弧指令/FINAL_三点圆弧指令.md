# FINAL：三点圆弧指令（CIRC / ARC）

## 交付摘要

实现工业 CIRC 语义的 `Type::ARC`：起点取上一段运动终点（链式种子 FK），指令存 Via+End；两步示教落盘；`CircularArcGeometry` 定圆采样；`ArcPlanner` 笛卡尔弧 + URDF IK → `jointTrajectoryRad`；预览/Run/指纹/属性面板贯通。

## 主要改动

| 模块 | 内容 |
|------|------|
| RobotKinematics | `CircularArcGeometry.h/.cpp` |
| RobotScene | `ArcInstruction`、via transform、Factory/schema、`ArcPlanner`、Executor |
| RobotWidget | ARC 按钮、两步示教、`appendArcInstructionFromPoses`、指纹/PlanJobPayload |
| docs | `docs/三点圆弧指令/*` |

## 质量

- 与 LINE 采样密度/lite/轨迹插帧对齐
- 面板改 Via 时清除 via transform 扩展，避免脏规划
- 品牌 MoveC 未做（见 TODO）
