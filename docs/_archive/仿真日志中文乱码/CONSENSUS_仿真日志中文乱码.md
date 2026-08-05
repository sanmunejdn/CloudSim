# CONSENSUS：仿真日志中文乱码与 DH 提示

## 验收标准

1. 中文界面 Run 时，懒加载与“仿真已开始”等不再显示 `????`。
2. DH 提示明确为“回退表未构建，主路径仍用 URDF IK”；仿真仍可启动。
3. `RobotWidget` **Debug|x64** 与 **Release|x64** 均编译通过。

## 技术方案

- 恢复 `RobotSimulationController.cpp` 中 `i18n(en, zh)` 的中文字面量（UTF-8 + 工程 `/utf-8`）。
- DH：仅文案澄清，不扩展自动 DH 分解算法。

## 边界

不修改 `decomposeDhFromOriginXyzRpy`；不声称 Fanuc URDF 可自动建 DH。
