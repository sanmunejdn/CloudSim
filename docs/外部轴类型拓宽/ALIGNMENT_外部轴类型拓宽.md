# ALIGNMENT — 外部轴类型拓宽

## 原始需求
将 CloudSim 外部轴从「单轴机器人地轨」扩展为可配置多轴（Translate/Rotate × RobotBase/Workpiece），贯通 FK、示教/PTP/LINE、Run。

## 项目理解
- 内核：`RobotExternalAxes` / `RobotTeachIk` / `RobotInstructionController`
- UI：`RobotExternalAxisSettingsWidget` / `RobotAxisControlWidget` / `RobotSimulationController`
- 存储：`basePlacementWorld=P0`；运行态 `externalAxisQ[]`；工件 `W0` 按 backend

## 边界
- 做：多轴配置、FK、采样+联立 IK、示教/规划/播放向量化
- 不做：Tesseract/MoveIt 依赖、闭链、品牌 E1/E2 导出

## 已确认
- Workpiece 必绑 `boundBackendId`；无空 backend 抽象架回退
- 单位：平移 mm、旋转 rad（UI deg）
