# ALIGNMENT — 工业相机插件二期（梅卡）

## 原始需求

二期落地梅卡 Mech-Eye；补齐 OpenCV 板检测与手眼「拍并检测」闭环；梅卡官方手眼作为 Ensemble 候选。

## 边界

| 纳入 | 排除 |
|------|------|
| MechEyeCamera + IP | 大恒/Basler、Host 契约改动 |
| OpenCV 棋盘格/ArUco | ethz 时间同步、无板点云手眼 |
| UI 品牌 MechMind | 品牌机器人官方 SDK |
| Mech HandEyeCalibration 候选 | 重做一期 UI |

## 假设

- Mech-Eye / OpenCV 以 `bin/SDK/MechEye`、`bin/SDK/opencv` + 宏接入；未装时可编过
- 无 OpenCV 时保留手填板位姿 / 演示样本
