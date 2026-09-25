# IndustrialCameraPlugin

> **文档导航**：[全库入口](../../README.md) · [全量目录](../../README.md) · [开发手册](../../开发手册/01-总览.md) · [产品索引](../README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

侧栏只注册一个「工业相机」页；内部用 `QTabWidget` 分隔 **相机** / **手眼标定** / **视觉抓取**。

视觉抓取依赖宿主 `IPluginRobotHost`（`minHostVersion` ≥ 1.55.0），规划参数读仿真「碰撞与规划」页。详见 [`docs/features/视觉抓取`](../../../docs/features/视觉抓取/README.md)。

日志统一走 `IPluginHostContext::logInfo` / `logError`。

依赖：`CloudSimPluginSDK`、`IndustrialCameraSDK`。

数据：`bin/x64(d)/resource/industrial_camera/`。

真机与位姿协议见 [`../IndustrialCameraSDK/DEVELOPER_GUIDE.md`](../IndustrialCameraSDK/DEVELOPER_GUIDE.md)。
