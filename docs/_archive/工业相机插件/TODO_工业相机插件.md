# TODO — 工业相机插件

## 一期（海康）缺配置

1. **安装海康 MVS** → `bin/SDK/Hikrobot-MVS/`，工程加 `CLOUDSIM_HAS_HIK_MVS` 与 Include/Lib
2. **安装 Mv3dRgbd** → `bin/SDK/Hikrobot-Mv3dRgbd/` + `CLOUDSIM_HAS_HIK_MV3D`
3. **机器人位姿桥**：控制器侧 TCP，`GET_POSE` / JSON（mm/deg）

## 二期（梅卡 / OpenCV）— 详见 [`../工业相机插件二期/TODO_工业相机插件二期.md`](../工业相机插件二期/TODO_工业相机插件二期.md)

1. Mech-Eye → `bin/SDK/MechEye/` + `CLOUDSIM_HAS_MECH_EYE`
2. OpenCV → `bin/SDK/opencv/` + `CLOUDSIM_HAS_OPENCV`（棋盘格「拍并检测」）

## 操作指引

- 插件目录：`bin/x64d/plugins/com.cloudsim.industrialcamera/`
- 数据目录：`bin/x64d/resource/industrial_camera/`
- 协议说明：`src/Plugins/IndustrialCameraSDK/DEVELOPER_GUIDE.md`
