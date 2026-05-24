# CloudSimPluginSDK 开发指南

## 定位

`CloudSimPluginSDK.dll` 是 **插件与宿主之间的唯一稳定 ABI**。插件工程只链接本 SDK，不得链接 `Widget.lib` / `RobotScene.lib`。

## 版本

- 宿主版本宏：`CLOUDSIM_PLUGIN_HOST_VERSION`（当前 `0x00010100` = 1.1.0，含 `sidePanelTabParent` / `registerSidePanelTab(const char*)` ABI）
- `IPluginDocument`：`documentId()`、`removeBackendObject()`（走 Host `IDataService`）
- `IPluginHostContext`：`importFileIntoActiveDocument()` → 宿主 `DocumentImportFacade::importFileIntoDocument`；`createPrimitiveMesh` / `registerTriangleMesh` → `registerAdoptedMesh`
- 清单 `plugin.json` 中 `minHostVersion` 使用字符串 `"1.0.0"`
- 运行时调用 `IPluginHostContext::hostVersion()` 比对

## 工具链约束

| 项 | 要求 |
|----|------|
| 平台 | x64 |
| 工具集 | v142（VS 2019） |
| Qt | 5.14.2_msvc2017_64（与 CloudSim 一致） |
| 运行时 | 将 `CloudSimPluginSDK.dll` 与插件 DLL 放在 exe 同目录或 `plugins/<id>/` |

## 插件契约

实现 `ICloudSimPlugin` 并导出 Qt 插件：

```cpp
class MyPlugin : public QObject, public ICloudSimPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
    Q_INTERFACES(ICloudSimPlugin)
    // pluginId(), displayName(), initialize(), shutdown()
};
Q_IMPORT_PLUGIN(MyPlugin) // 仅静态测试时需要
```

## 清单 `plugin.json`

与 exe 同级目录：`plugins/<id>/plugin.json`

```json
{
  "id": "com.example.hello",
  "name": "Hello",
  "version": "1.0.0",
  "minHostVersion": "1.0.0",
  "library": "HelloPlugin.dll",
  "enabled": true
}
```

## 宿主 API 摘要

| API | 说明 |
|-----|------|
| `registerSidePanelTab` | 右侧面板新页签（与 Workspace/AI 并列，**推荐**，避免与仿真 Dock 重叠） |
| `unregisterSidePanelTab` | 移除页签（`shutdown` 时调用） |
| `registerDockWidget` | 浮动 Dock（勿用 `Right`，会与工作区重叠） |
| `registerMenuPath` / `registerAction` | 菜单与动作 |
| `importFileIntoActiveDocument` | 活动文档导入文件；返回 root `backendId`（UTF-8） |
| `createPrimitiveMesh` | box/cylinder/cone/sphere → Host 注册 + OSG |
| `registerBackendType` | 自定义 `className`（`PluginDelegatedBackend`） |
| `registerTriangleMesh` | 三角 soup → `registerAdoptedMesh` |
| `enqueueJob` / `invokeOnUiThread` | 线程边界 |

宿主实现细节：[`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)。

## 线程

- `initialize` / `shutdown` / UI 回调：**UI 线程**
- 重 CPU 工作：`enqueueJob`，完成后在 UI 线程写场景

## 示例

见 [`HelloPlugin/DEVELOPER_GUIDE.md`](../HelloPlugin/DEVELOPER_GUIDE.md)（`plugin.json`、侧栏页签、`createPrimitiveMesh` 流程）。
