# FINAL — 外部轴类型拓宽

## 交付摘要
CloudSim 外部轴从单轴地轨扩展为多轴全链路：配置（Translate/Rotate × RobotBase/Workpiece）、FK（P0/W0）、TeachIk 采样+联立、PlanResult/指令 CSV、轴控与 Run 插帧。

## 主要改动面
- `RobotExternalAxes.*`、`RobotTeachIk.*`、`RobotInstructionController.*`、`ExternalAxisSearchService.*`
- `DocumentPage` / `IRobotDocumentHost` / `MainWindowRobotHost`
- `RobotExternalAxisSettingsWidget`、`RobotAxisControlWidget`、`RobotSimulationController`
- `docs/外部轴类型拓宽/*`、`DEVELOPER_GUIDE`（RobotScene/RobotWidget）
