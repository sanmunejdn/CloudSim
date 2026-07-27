# ACCEPTANCE — 非刚性配准轨迹算子

## 已完成

- [x] `TrajectoryOpKind::NonRigidRegistration` + `NonRigidRegistrationParams`
- [x] `INonRigidTrajectoryWarp` + `RobotSceneNonRigidTrajectoryWarp`
- [x] `TrajectoryNonRigidWarp.cpp`：2×2 SPARE 分发、mesh/PC 双绑定、scope 写回
- [x] Builtins 四件套 + JSON + 注册 + vcxproj
- [x] UI 调色板 / 参数面板 / Apply 路径
- [x] `TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md` 更新

## 待手工验证（T-NR1~7）

需在 Visual Studio x64 编译后，于轨迹编辑页拖入「非刚性配准纠正」块测试。
