# PlcCommPlugin

将 **PlcCommUI** 注册为 CloudSim 右侧栏 **PLC** 页签。

## 依赖

| 库 | 说明 |
|----|------|
| `CloudSimPluginSDK` | `ICloudSimPlugin` / `IPluginHostContext` |
| `PlcCommUI` | `createPlcCommWidget()` |

运行时 `bin/x64(d)/` 需已有 `PlcCommSDK.dll`、`PlcCommUI.dll`、`plctag.dll`（由 PlcCommSDK 工程 PostBuild 复制）。

## 部署

```text
bin/x64(d)/plugins/com.cloudsim.plccomm/
  plugin.json
  PlcCommPlugin.dll
```

## 清单

- `id`：`com.cloudsim.plccomm`（与 `pluginId()` 一致）
- `enabled`：`true` 时启动加载

## 语言

与主窗口 **设置 → 语言** 同步：`useChinese()` 默认中文；切换时 `onLanguageChanged` 更新侧栏标题（`PLC 通讯` / `PLC`）并调用 `PlcCommWidget::applyLanguage()`。
