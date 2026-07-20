# CONSENSUS — 转换工件型轨迹算子

## 验收标准

| ID | 场景 | 期望 |
|----|------|------|
| T-WH1 | 注入 TCP + 外部 TCP=单位，输入点=参考位姿 | 输出接近外部 TCP |
| T-WH2 | 改外部 TCP xyz/rpy | 整条输出随之刚体变化 |
| T-WH3 | 未捕获到机器人 TCP | processPath 失败，errMsg 明确 |
| T-WH4 | enableSpeedTransform 开/关 | 开：速度按弧长比；关：速度不变 |
| T-WH5 | Preview / Apply / Undo | 与 Translate 同类位姿块一致 |

## 技术方案

1. `ToWorkpieceInHand` 原子块；公式 \(B_T_{Eout}=B_T_{TCP}\,(W_f_T_{Ei})^{-1}\)（\(F_T_W=I\)）
2. Context 增加 `workpieceReferenceInBase`；Engine setter；Session 捕获后注入
3. 参数：`externalTcp*` + `enableSpeedTransform`

## 约束

- 位姿单位：mm + ZYX deg（与 UnifiedTrajectory / RigidTransform 一致）
- 依赖方向：RobotWidget → RobotScene → TrajectoryAlgorithm
