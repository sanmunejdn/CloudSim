# IndustrialCameraSDK

工业相机领域 DLL：`ICamera`、海康/梅卡 stub 或真 SDK、模拟相机、OpenCV 板检测、手眼 Ensemble、位姿源。

## 启用真机 / OpenCV

| 宏 | 部署目录 | 说明 |
|----|----------|------|
| `CLOUDSIM_HAS_HIK_MVS` | `bin/SDK/Hikrobot-MVS/` | 海康 2D |
| `CLOUDSIM_HAS_HIK_MV3D` | `bin/SDK/Hikrobot-Mv3dRgbd/` | 海康 3D |
| `CLOUDSIM_HAS_MECH_EYE` | `bin/SDK/MechEye/` | 梅卡 Mech-Eye |
| `CLOUDSIM_HAS_OPENCV` | `bin/SDK/opencv/` | 棋盘格板检测 |

在 `IndustrialCameraSDK.vcxproj` 的 `PreprocessorDefinitions` 与 Include/Lib 中按需追加。未定义宏时：对应品牌连接/检测返回明确中文错误；可用 `Simulated` 调试。

## 手眼

- Eigen 五法 + 残差择优：`solveHandEyeEnsemble`
- 梅卡官方候选：`MechOfficialHandEyeSession` → `mergeHandEyeCandidate`
- 板位姿：`detectBoardPose`（OpenCV）或 UI 手填

## 真实机器人位姿协议

TCP 文本：客户端连上后发 `GET_POSE\n`，服务端回一行：

```json
{"x":100.0,"y":200.0,"z":300.0,"rx":0.0,"ry":0.0,"rz":90.0}
```

单位：mm；rx/ry/rz：度，ZYX 内旋。
