# ACCEPTANCE — 外部轴类型拓宽

| 项 | 结果 |
|----|------|
| 多轴配置存盘 | 代码路径已通（JSON axes 新字段 + 旧兼容） |
| 轴控 RobotBase/Workpiece | `applyAxisControlExternalPose` 分流 compose |
| Plan/示教 CSV + Qs | InstructionController + writeExternalAxisPlanToInstruction |
| Run 插值 | `applyExternalAxisFromPlan` 向量 lerp |
| 编译 | `RobotScene` + `RobotWidget` Debug|x64 已通过 |
| 二期 workingFrame / REP | `resolveWorkingFrameId` + Offset 捕获；规划外层 Workpiece 采样；拖动 `solveTcpDragTeachIk` REP；设置页工作架下拉；`workingTcp*` 扩展键 |

场景手测清单：机器人地轨、机器人转台、工件 1–2 轴变位机、混合 1 轨+1 工件转；二期：工件转台拖动联立、`workingFrameId` 偏置子 backend。
