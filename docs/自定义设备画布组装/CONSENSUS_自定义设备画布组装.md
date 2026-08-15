# CONSENSUS — 自定义设备画布组装

## 需求与验收

1. 画布上 Link 块对应 Mesh/Brep；连线创建 Joint（Revolute/Prismatic）
2. 有向树、单一固定根；FK：`W_child = W_parent * Motion(q) * Rest`
3. 轴控 DOF 与 Joint 列表一致；存盘 `links[]`/`joints[]`；旧 `deviceAxes` 可读
4. Debug|x64 与 Release|x64 相关工程编译通过

## 技术方案

- Data：`CustomDeviceLink` / `CustomDeviceJoint`；`syncAxesFromJoints()`
- FK：`CustomDeviceKinematics::applyQ` 有 joints 时走树写子件世界并刷新 Follow local
- UI：`CustomDeviceAssemblyCanvasWidget` + 组装对话框（替换扁平轴主路径）

## 不做

闭环、Mate、URDF、Web 画布、联立 IK
