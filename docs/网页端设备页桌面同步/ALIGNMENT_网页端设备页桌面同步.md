# ALIGNMENT — 网页端设备页桌面同步

## 原始需求

右栏「机器人」改为「设备」；顶栏切换机器人 / 自定义设备；自定义设备侧姿态库 + DI→姿态绑定对齐桌面；左栏设备收成目录/组装。

## 边界

- 做：RightDock 结构、DeviceCommandPanel、轴控双目标、左栏瘦身、文档与 web 构建
- 不做：DevicePoseMotionPlayer 插值、组装画布增强、DEV_AXIS 插入栏

## 对齐对象

桌面 `RobotSimulationDockWidget` + `DeviceCommandPageWidget`；数据仍为 `poseSignalBindings` + Host 上升沿。
