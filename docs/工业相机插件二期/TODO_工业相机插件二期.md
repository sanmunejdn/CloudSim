# TODO — 工业相机插件二期

## 缺配置（本机）

1. ~~**Mech-Eye SDK**~~ → 已 junction：`bin/SDK/MechEye` ← `C:\Mech-Mind\Mech-Eye SDK-2.5.4`；工程已开 `CLOUDSIM_HAS_MECH_EYE`
2. ~~**OpenCV**~~ → 已部署 `bin/SDK/opencv/opencv/build`（4.10.0）；宏 `CLOUDSIM_HAS_OPENCV` 已开
3. **海康 MVS（2D）** → 本机未装；见 `bin/SDK/Hikrobot-MVS/README_DOWNLOAD.txt` 官网下载
4. ~~**海康 Mv3dRgbd**~~ → 头/库/DLL 已组装到 `bin/SDK/Hikrobot-Mv3dRgbd`；`CLOUDSIM_HAS_HIK_MV3D` 真机取流仍为 stub
5. 真机联调：枚举 IP → 连接 → 彩色/点云落盘 → 手眼「拍并检测」≥6 组

## 可选

- ArUco 字典路径补全（当前拍并检测以棋盘格为主）
- 官方标定板型号做成配置项（当前默认 `CGB_20`）

## 操作指引

- 插件：`bin/x64d/plugins/com.cloudsim.industrialcamera/`
- 数据：`bin/x64d/resource/industrial_camera/`
- SDK 总览：`bin/SDK/README_工业相机SDK.md`
- SDK 说明：`src/Plugins/IndustrialCameraSDK/DEVELOPER_GUIDE.md`
