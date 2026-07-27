# ACCEPTANCE — 工业相机插件二期（梅卡）

> 对应：`docs/工业相机插件二期/` ALIGNMENT / CONSENSUS / DESIGN / TASK

## 验收对照

| # | 标准 | 结果 | 说明 |
|---|------|------|------|
| 1 | Mech SDK + 宏：枚举含 IP；可连；彩色+点云；落盘 `captures/` | 待真机 | 代码路径已就绪；本机未装 Mech-Eye，需 `bin/SDK/MechEye` + `CLOUDSIM_HAS_MECH_EYE` |
| 2 | 无 Mech SDK：选 MechMind 连接失败提示明确；不崩；Simulated/海康不受影响 | **通过** | Debug\|x64 已编过；无宏时 `MechEyeCamera::connect` 返回中文错误 |
| 3 | OpenCV：「拍并检测」解出板位姿入样本；≥6 可 Ensemble | 待环境 | `BoardDetector` + UI 已接线；需 `bin/SDK/opencv` + `CLOUDSIM_HAS_OPENCV` |
| 4 | 无 OpenCV：手填板位姿 / 演示样本仍可用 | **通过** | 「手填添加样本」「填充演示样本」保留；拍并检测返回明确错误文案 |
| 5 | UI 单宿主「工业相机」+ 内嵌 Tab；日志进宿主 | **通过** | 未改 Dock 结构；日志仍 `host->logInfo/Error` |

## 编译验证

- `IndustrialCameraSDK` Debug\|x64 → `bin/x64d/IndustrialCameraSDK.dll`
- `IndustrialCameraPlugin` Debug\|x64 → `bin/x64d/plugins/com.cloudsim.industrialcamera/`

## 交付清单

| 项 | 路径 |
|----|------|
| MechEyeCamera | `source/mech/MechEyeCamera.*` |
| BoardDetector | `inc/BoardDetector.h` + `source/calib/BoardDetector.cpp` |
| MechOfficial | `inc/MechOfficialHandEye.h` + `source/calib/MechOfficialHandEye.cpp` |
| Factory/UI | Factory 枚举/创建 MechMind；品牌 Combo；拍并检测；Ensemble 合并 MechOfficial |
| 文档 | 本目录 ALIGNMENT→ACCEPTANCE；一期 TODO / ARCHITECTURE 已更新 |
