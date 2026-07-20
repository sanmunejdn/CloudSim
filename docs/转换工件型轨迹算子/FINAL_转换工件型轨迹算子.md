# FINAL — 转换工件型轨迹算子

## 交付摘要

新增轨迹原子块 `ToWorkpieceInHand`（转换工件型）：工具型末端轨迹映射为工件型（手持工件对固定外部 TCP）。

- 外部 TCP：算子参数（mm + ZYX deg）
- 参考位姿 \(B_T_{W_f}\)：`TrajectoryEditSession` 捕获当前工具 TCP → Engine → Context
- 公式对齐 HPL `HPLTPToWorkpieceInHandStrategy`（\(F_T_W=I\)）
- 可选弧长速度变换写回 `speedMmPerSec`

## 主要改动路径

- `TrajectoryPipelineTypes.h` / `TrajectoryOpParamsParse.*`
- `TrajectoryOpExecutionContext.h` / `TrajectoryPipelineEngine.*`
- `TrajectoryEditSession.*`
- `ops/ToWorkpieceInHand/*` + `ops/ToWorkpieceInHand.json`
- `TrajectoryOpBuiltinsRegister.cpp` / vcxproj / filters
- 文档：`docs/转换工件型轨迹算子/`、两份 DEVELOPER_GUIDE
