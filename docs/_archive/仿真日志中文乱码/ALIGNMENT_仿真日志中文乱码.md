# ALIGNMENT：仿真日志中文乱码与 DH 提示

## 原始需求

排查运行日志：

1. `DH rows not built: joint 'joint_2' ... pitch过大，r20=1`
2. `????????????? 16/6521 ??????????????`
3. `??????????`

并修复中文乱码。

## 项目理解

- 主 IK 路径：`RobotScene` + URDF 数值 IK（见 `RobotKinematics` DEVELOPER_GUIDE：DH 仅为缺 TCP 上下文时的回退）。
- 仿真 Run Info 走 `MainWindow::i18n(en, zh)`，中文界面取 `zh`。
- `RobotSimulationMath::buildDhRowsFromUrdf` 要求关节 origin 的 RPY 可拆成修正 DH（近似要求 `pitch≈0`，即 `|r20|=|-sin(pitch)|≤1e-3`）。

## 边界

| 在范围 | 不在范围 |
|--------|----------|
| 解释上述日志成因 | 为 Fanuc 等机型实现完整 URDF→DH |
| 修复 `RobotSimulationController.cpp` 中被 `?` 破坏的中文 i18n | 全仓库所有注释乱码 |
| 将 DH 提示改为“回退不可用、主路径仍用 URDF IK” | 改动 IK/规划算法 |

## 决策

- DH 失败为预期（如 R-2000iC `joint_2` 常有 pitch≈±90°）→ 不清零仿真，仅无 DH 回退。
- 乱码根因是源文件中文被替换成 `?`，不是控制台编码。
