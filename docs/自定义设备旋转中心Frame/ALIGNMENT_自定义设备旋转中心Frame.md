# ALIGNMENT — 旋转副运动中心绑 Frame

## 决策

只做 **自定义坐标系**：旋转中心绑定已有 `FrameBackendData`（或连杆几何原点）。

## 行为

- 「旋转中心」下拉：
  - （无）
  - **模型：…** — 组装画布上各连杆几何自身坐标系（原点）
  - **坐标系：…** — 场景已有 Frame
- 存 `motion.motionCenterFrameBackendId`（Frame 或模型 Backend id）
- **组装/提交**：将该 Backend 原点变到父连杆局部 → `originMm`（`rebakeRotateJointOriginsFromFrames`）
- **轴控 / 运行**：
  - FK 用父连杆局部常数 `originMm`（链式枢轴随上游关节动）
  - `applyQ` 末尾 `syncMotionCenterFramesFromOrigins`：`W_frame = W_parentLink × T(originMm)`，使场景坐标系显示与 FK 一致
  - 默认不在 `applyQ` 中从 Frame 反烘焙 `originMm`（`rebakeOriginsFromSceneFrames=false`）

## 机器人挂载

`updateMountedDeviceWorldFromRobotTcp` 更新设备 `W0` 后执行 `applyQ`（含旋转中心 Frame 回写与连杆 FK），再由 `refreshCustomDevicesFollowingKinematicsTargets` flush 视觉。

## 相关

- [运动副设计](../运动副/DESIGN_运动副.md) §6.2–6.3
- [机器人法兰挂载](../自定义设备机器人挂载/DESIGN_自定义设备机器人挂载.md)
