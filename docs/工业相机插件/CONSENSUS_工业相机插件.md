# CONSENSUS — 工业相机插件（海康一期）

## 需求与验收

1. 枚举/按 IP 连接海康 2D（MVS）与 3D（Mv3dRgbd）；无 SDK 时插件不崩并提示
2. 单帧取 2D（及深度/点云）；预览；保存到 `resource/industrial_camera/captures/`
3. 点云可 `importFileIntoActiveDocument`
4. 手眼：眼在手上/外；手动或 TCP/JSON 读末端位姿；≥6 样本；多算法择优；结果写入 `calibrations/`
5. 内参写入 `cameras/<serial>/`；meta 含 ip+serial
6. UI：双侧栏 Tab，阶段流清晰

## 技术方案

| 模块 | 说明 |
|------|------|
| IndustrialCameraSDK.dll | `ICamera`、Factory、手眼 Ensemble、位姿源；仅 Eigen（OpenCV/MVS 可选宏） |
| IndustrialCameraPlugin.dll | Qt 双 Tab；链 PluginSDK + IndustrialCameraSDK |
| 海康适配 | `HikMvsCamera` / `HikMv3dCamera`；无厂商头时走 stub 提示；`SimulatedCamera` 调试 |
| 手眼 | MotionPairFilter + Tsai/Park/Horaud/Andreff/Daniilidis(DQ) + 残差择优 |
| 真实位姿 | `RealRobotPoseSource`：TCP 收一行 JSON `{"x","y","z","rx","ry","rz"}`（mm/deg） |

## 技术约束

- v142、C++17、UTF-8 BOM、头卫 `INDUSTRIALCAMERASDK_*` / `INDUSTRIALCAMERAPLUGIN_*`
- 插件不链 Widget/RobotScene
- 厂商 SDK 路径：`bin/SDK/Hikrobot-MVS`、`Hikrobot-Mv3dRgbd`、`opencv`（可选）

## 集成

- `plugin.json` → `plugins/com.cloudsim.industrialcamera/`
- 点云导入走 `IPluginHostContext::importFileIntoActiveDocument`
