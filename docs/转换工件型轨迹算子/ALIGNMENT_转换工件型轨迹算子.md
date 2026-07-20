# ALIGNMENT — 转换工件型轨迹算子

## 原始需求

在轨迹流水线中新增原子块 `ToWorkpieceInHand`，参考 HPL `HPLTPToWorkpieceInHandStrategy`：将工具型末端轨迹转换为工件型（手持工件对固定外部 TCP）末端轨迹。

## 边界

- 外部 TCP：算子参数（mm + ZYX deg）
- 工件固定参考 \(B_T_{W_f}\)：由 `TrajectoryOpExecutionContext` 注入当前机器人 TCP（基座系）；不走 HPL observer
- 法兰→工件：第一版单位阵；不做标定 UI
- 速度变换：可选；开启时按弧长比缩放 `speedMmPerSec`
- 启用：复用 `TrajectoryOpDescriptor.enabled`

## 集成

- Builtins 四件套 + Context/Engine 注入
- `TrajectoryEditSession` 在 execute 前经 `tryCaptureCurrentRobotTcpPose` 注入
- Builtins 不链 RobotWidget
