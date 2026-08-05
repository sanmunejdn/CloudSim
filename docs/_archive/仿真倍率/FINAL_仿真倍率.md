# FINAL：仿真倍率

## 交付

- `RobotProgramExecutor`：播放倍率 + 虚拟时钟（运动段 / WAIT / `motionSegmentProgress01`）
- `SimulationCommandWidget`：Run/Stop 旁「仿真倍率」下拉
- `RobotSimulationController`：启动与运行中同步倍率

## 验证

重启 CloudSim 后：选 `2×` 再 Run，段播放应明显加快；运行中切到 `0.5×` 应减速且不跳点。
