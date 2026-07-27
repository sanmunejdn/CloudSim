# FINAL：外部轴联动求解

## 交付摘要

实现了机器人实例级外部轴配置（一期地轨）、仿真 Dock 配置 UI、工程持久化，以及「先配置后联动」门禁下的 ExternalAxisSearch（1D 网格 + 可选联立 DLS）。同步优化了 RobotKinematics（闭式 MDH、解析位置雅可比、棱柱关节）与 TeachIk 外轴 DOF。

后续补齐：**存储 P0 / 运行时 `externalAxisQMm` + FK 合成 `P_eff`**；TCP 拖动联立；PTP/LINE/Run 规划回传 `externalAxisQ`；**点云/Run 播放时臂轨迹与地轨 qe 同步插帧**（2026-07-23）。

## 关键路径

| 区域 | 路径 |
|------|------|
| 配置模型 | `RobotScene/inc/RobotExternalAxes.h` |
| 合成 / 平移布局 | `RobotScene/source/RobotExternalAxes.cpp`（Mat4 平移在 `[3,7,11]`） |
| 搜索服务 | `RobotScene/inc/ExternalAxisSearchService.h` |
| UI 配置 | `RobotWidget/inc/RobotExternalAxisSettingsWidget.h` |
| 轴控滑条 | `RobotAxisControlWidget` + `RobotSimulationController::applyAxisControlExternalPose` |
| IK | `RobotScene/source/RobotTeachIk.cpp`（联立 / 拖动协调） |
| 规划 | `RobotInstructionController.cpp`（PTP/LINE + `applyLastIkExternalAxisToPlan`） |
| 播放 | `RobotProgramExecutor` + `RobotSimulationController::onRobotSimulationTick` |
| DH | `RobotKinematics/source/SerialLinkKinematics.cpp` |
| Op | `TrajectoryAlgorithmBuiltins/ops/ExternalAxisSearch/` |
| 6A 文档 | `CloudSim/docs/外部轴联动求解/` |

## 设计要点

- 门禁：`hasEnabledExternalAxes`；未启用则 Search 直接返回
- **存储**：`basePlacementWorld` = P0（不烘焙轨位）；运行态 `externalAxisQMm`
- **FK**：`P_eff = P0 * Trans(q·axis)`（`composeBasePlacementWithExternalAxis`）
- 运动学：`p_arm = p_target - q_e * axis` / 联立时 `p_eff = FK + q_e * axis`
- 配置存于 `robotKinematicsInstances[].externalAxes`
- **播放**：关节走 `jointTrajectoryRad`（≥2）或起止 lerp；外轴按段进度 lerp；`durationSec` 含地轨行程

## 编译

`CloudSim.sln` `/t:RobotScene;RobotWidget` Debug\|x64（2026-07-23 已通过）。
