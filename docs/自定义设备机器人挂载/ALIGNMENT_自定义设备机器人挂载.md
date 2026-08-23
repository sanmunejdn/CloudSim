# ALIGNMENT — 自定义设备机器人法兰挂载

## 目标

自定义设备（Link/Joint 组装）作为机器人末端执行器，刚性跟随法兰；设备内部轴仍由 `applyQ` / `DeviceAxisInstruction` 驱动。

## 约束

- 不与 `RobotToolFrame` / PTP·LINE IK 联动（V1）
- 设备根不用 `FollowAttachment` 挂法兰（与 `applyQ` 写 W0 冲突）
- 安装接口用场景 `FrameBackendData` 对齐；冻结 `T_flange_device` 持久化在 `CustomDeviceRobotMount` 组件

## 验收

1. 组装 → 对齐 Frame → 挂载 → 拖机器人关节时执行器随法兰移动
2. 挂载态轴控、程序 `DeviceAxis`、DI→姿态正常
3. 解除挂载回 `baseWorldW0`；工程保存/加载一致
4. Web `POST .../mount` / `.../unmount` 与桌面等价
