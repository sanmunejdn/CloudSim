# CONSENSUS — 自定义设备

## 需求描述

1. 新增后端类型 `CustomDeviceBackendData`（catalog `CustomDevice`），作为设备聚合根
2. 导入的 Model/Brep 等可挂为子件（`setParent` + Follow），改设备根位姿时整体跟随
3. 设备上定义单轴平移或旋转：方向/轴、旋转中心、上下限/home；结构预留多轴
4. 仿真轴控页可切换「机器人 | 自定义设备」并拖动预览
5. 工程保存/加载恢复设备、子树、运动参数与 `q`

## 技术方案

- 契约：`BackendTypeIds` 三键 + `isCustomDeviceClassName`
- 运动字段在 `saveDerivedJson`：`axes`、`q`、`baseWorldW0`；`geometry.kind=customDevice`
- FK：`W = W0 * Π T_i(q_i)`，复用 `RobotExternal::makeAxisMotionColumnMajor`（RobotScene 适配层）
- Visual：设备根 RGB 轴示意（同 Frame 风格）
- 轴控：目标下拉；选设备时用外轴滑条语义显示设备 DOF

## 验收标准

1. 创建设备、挂接子件后，移动设备根则子件跟随
2. 定义平移/旋转后，轴控切换到该设备，滑条驱动正确
3. 存盘重开后状态恢复；切回机器人行为不变
4. Debug|x64 与 Release|x64 相关工程编译通过

## 不做

联立 IK、多轴 UI、URDF 导出、Web、3D 拾取中心
