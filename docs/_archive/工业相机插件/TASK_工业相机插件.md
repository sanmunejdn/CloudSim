# TASK — 工业相机插件

## T1 Align 文档
- 出：ALIGNMENT/CONSENSUS/DESIGN
- 验：决策表与计划一致

## T2 SDK 骨架
- 出：IndustrialCameraSDK 工程 + ICamera/Types/Factory/Stub
- 验：可编 DLL

## T3 Hik 2D / 3D
- 出：HikMvsCamera、HikMv3dCamera（stub 或真 SDK）
- 验：无 SDK 时 lastError 明确；有模拟相机可取帧

## T4 插件 UI + 落盘
- 出：双 Tab + CameraResourceStore
- 验：路径落在 resource/industrial_camera

## T5 手眼 Ensemble
- 出：多算法 + 向导阶段 UI
- 验：≥6 样本可出 best + scores

## T6 RealRobotPoseSource
- 出：TCP/JSON 读位姿
- 验：协议文档 + 能写入样本

## T7 验收文档
- 出：ACCEPTANCE/FINAL/TODO + ARCHITECTURE 一行
