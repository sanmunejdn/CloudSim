# CONSENSUS — 工业相机插件二期

## 验收

1. Mech SDK+宏：IP 枚举/连接、彩色+点云、落盘、导入
2. 无 Mech SDK：明确错误、不崩
3. OpenCV+宏：拍并检测解板位姿；≥6 样本 Ensemble
4. 无 OpenCV：手填/演示仍可用
5. 单侧栏内嵌 Tab + 宿主日志不变

## 方案

- `MechEyeCamera` + `CLOUDSIM_HAS_MECH_EYE`
- `BoardDetector` + `CLOUDSIM_HAS_OPENCV`
- Ensemble 追加 `MechOfficial`（有梅卡 SDK 时）
