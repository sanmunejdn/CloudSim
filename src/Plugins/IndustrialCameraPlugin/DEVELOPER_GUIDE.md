# IndustrialCameraPlugin

> **文档导航**：[全库入口](../../../../docs/README.md) · [全量目录](../../../../docs/全量目录.md) · [开发手册](../../../../docs/开发手册/01-总览.md) · [产品索引](../../../docs/README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

侧栏只注册一个「工业相机」页；内部用 `QTabWidget` 分隔 **相机** / **手眼标定**。

日志不再使用面板内调试窗口，统一走 `IPluginHostContext::logInfo` / `logError`（宿主日志页）。

依赖：`CloudSimPluginSDK`、`IndustrialCameraSDK`。

数据：`bin/x64(d)/resource/industrial_camera/`。

真机与位姿协议见 [`../IndustrialCameraSDK/DEVELOPER_GUIDE.md`](../IndustrialCameraSDK/DEVELOPER_GUIDE.md)。
