# FINAL：仿真日志中文乱码与 DH 提示

## 结论

| 日志 | 原因 | 处理 |
|------|------|------|
| `joint_2` DH 不可分解（r20=1） | URDF origin pitch≈−90°，当前自动 DH 要求 pitch≈0 | 文案改为“回退不可用，主路径仍用 URDF IK”；算法未改 |
| `???? 16/6521 ????` | 源码中文被破坏为 `?` | 已恢复为「懒加载规划：启动时已规划 %1/%2 条…」 |
| `??????????` | 同上 | 已恢复为「仿真已开始。」 |

## 编译

- Debug\|x64：通过（`bin\x64d\RobotWidget.dll`）
- Release\|x64：通过（`bin\x64\RobotWidget.dll`）

## 变更文件

- `CloudSim/src/UI/RobotWidget/source/RobotSimulationController.cpp`
- `CloudSim/src/UI/RobotWidget/tools/restore_i18n_zh.py`
