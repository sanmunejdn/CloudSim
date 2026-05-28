# PlcCommPlugin 开发指南

CloudSim 动态插件：将 **PlcCommUI** 注册为右侧栏 **PLC 通讯** 页签。

## 依赖

| 库 | 用途 |
|----|------|
| `CloudSimPluginSDK` | `ICloudSimPlugin`、`IPluginHostContext` |
| `PlcCommUI` | `createPlcCommWidget()` |

## 构建顺序

```text
PlcCommSDK → PlcCommUI → PlcCommPlugin
CloudSimPluginSDK（并行）
```

在 `CloudSim.sln` 中生成 **PlcCommPlugin** 即可（ProjectReference 拉取依赖）。

## 运行时布局

```text
bin/x64(d)/
  PlcCommSDK.dll
  PlcCommUI.dll
  plctag.dll
  CloudSimPluginSDK.dll
  plugins/com.cloudsim.plccomm/
    plugin.json
    PlcCommPlugin.dll
```

插件 DLL 仅含薄封装；**必须**保证 exe 同目录已有 `PlcCommUI.dll`、`PlcCommSDK.dll`、`plctag.dll`。

## plugin.json

```json
{
  "id": "com.cloudsim.plccomm",
  "name": "PLC",
  "version": "1.0.0",
  "minHostVersion": "1.1.0",
  "library": "PlcCommPlugin.dll",
  "enabled": true
}
```

- `id` 与 `PlcCommPlugin::pluginId()` 一致
- `enabled: false` 可禁用加载

## 生命周期

1. `PluginManager` 加载 `plugins/com.cloudsim.plccomm/`
2. `initialize(host)`：`createPlcCommWidget()` → `registerSidePanelTab("PLC 通讯", widget)`
3. `onLanguageChanged` → `applyLanguage()` + `setSidePanelTabTitle`
4. `shutdown()` → `unregisterSidePanelTab`

## 语言

| 项 | 中文 | 英文 |
|----|------|------|
| 侧栏标题 | PLC 通讯 | PLC |
| 插件日志 | PLC 通讯插件已加载。 | PLC comm plugin initialized. |

## 与 PlcCommSDK 的边界

- 插件 **不**直接链接 `libplctag`
- 插件 **不**应包含 PLC 业务逻辑；新功能优先加在 SDK/UI

## 相关文档

- SDK：[`../PlcCommSDK/DEVELOPER_GUIDE.md`](../PlcCommSDK/DEVELOPER_GUIDE.md)
- UI：[`../PlcCommUI/DEVELOPER_GUIDE.md`](../PlcCommUI/DEVELOPER_GUIDE.md)
- 插件系统：[`../../../ARCHITECTURE_SUMMARY.md`](../../../ARCHITECTURE_SUMMARY.md) §10
