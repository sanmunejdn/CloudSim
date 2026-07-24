# FINAL — 工业相机插件（海康一期）

## 交付摘要

在 CloudSim 中新增：

- `IndustrialCameraSDK`：通用 `ICamera`、海康 MVS/Mv3d stub、模拟相机、手眼多算法 Ensemble、手动/TCP 位姿源
- `IndustrialCameraPlugin`：双侧栏（相机 / 手眼）、资源落盘 `resource/industrial_camera/`
- 工程已加入 `CloudSim.sln`

## 使用提示

1. 无海康 SDK 时选 **Simulated**，IP `127.0.0.1`，可拍照保存与导入点云
2. 手眼：配置 → 采集 →「填充演示样本」→ 求解，查看算法对比与 `calibrations/` 结果
3. 真机：安装 MVS/Mv3dRgbd 到 `bin/SDK/`，工程加宏后重编 SDK

## 与计划差异

- OpenCV 未引入：板检测改为手填 `T_cam_board`（+ 演示样本合成）；真板检测待 OpenCV 部署后接
- Camodocal 以 Daniilidis 对偶四元数 Eigen 实现替代整仓移植
