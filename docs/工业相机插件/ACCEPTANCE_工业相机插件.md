# ACCEPTANCE — 工业相机插件（海康一期）

## 构建

| 项 | 结果 |
|----|------|
| IndustrialCameraSDK Debug\|x64 | 通过 → `bin/x64d/IndustrialCameraSDK.dll` |
| IndustrialCameraPlugin Debug\|x64 | 通过 → `bin/x64d/plugins/com.cloudsim.industrialcamera/` |

## 功能对照

| 验收项 | 状态 | 说明 |
|--------|------|------|
| 海康 2D/3D 真机 | 条件满足 | 需 `CLOUDSIM_HAS_HIK_MVS` / `CLOUDSIM_HAS_HIK_MV3D` + SDK；未装时明确错误 |
| 模拟相机 IP/枚举/取图/点云 | 可通过 UI | 品牌 Simulated |
| 落盘 resource/industrial_camera | 已实现 | captures/cameras/calibrations |
| 双 Tab UI | 已实现 | 工业相机 + 手眼标定 |
| 多算法 Ensemble | 已实现 | Tsai/Park/Horaud/Andreff/Daniilidis + 择优；演示样本按钮 |
| 手动位姿 | 已实现 | |
| RealRobotPoseSource TCP/JSON | 已实现 | 见 SDK DEVELOPER_GUIDE |
| 梅卡 | 二期 | 未做 |

## 文档

ALIGNMENT / CONSENSUS / DESIGN / TASK / FINAL / TODO 已落盘 `docs/工业相机插件/`。
