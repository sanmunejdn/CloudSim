# ACCEPTANCE — 转换工件型轨迹算子

| ID | 场景 | 状态 | 备注 |
|----|------|------|------|
| T-WH1 | 注入 TCP + 外部 TCP=单位，输入=参考位姿 → 输出接近外部 TCP | 待手工 | 公式退化 |
| T-WH2 | 改外部 TCP xyz/rpy → 输出刚体跟随 | 待手工 | |
| T-WH3 | 未捕获 TCP → processPath 失败，errMsg「缺少当前机器人 TCP 参考位姿」 | 代码已实现 | Session 注入失败时不设标志 |
| T-WH4 | enableSpeedTransform 开/关 | 代码已实现 | 开：乘弧长比；关：不动 speed |
| T-WH5 | Preview / Apply / Undo | 待手工 | capabilities=PreviewPoseTransform |

## 编译自检

- [x] `TrajectoryAlgorithmBuiltins` x64 Release → `.lib` 成功
- [x] `TrajectoryPipelineEngine.cpp` / Context 变更可编译（RobotScene 全量链接受环境 lib 路径影响，与本改动无关）

## 实现核对

- [x] `TrajectoryOpKind::ToWorkpieceInHand` + Params + Parse
- [x] Context / Engine / Session 注入
- [x] Op + OpConfig + JSON + 注册 + vcxproj
- [x] DEVELOPER_GUIDE 更新
