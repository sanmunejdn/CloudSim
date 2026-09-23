# AiWidget 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `AiWidget.dll` |
| 职责 | 桌面 AI 助手坞：对话 Dock、确认面板、LLM 设置；协调 PluginHost AI 运行时 |

## 2. 边界

- **依赖**：CloudSimPluginHost AI 栈、CloudSimAiSDK
- **不负责**：网页 AI 坞（在 `cloudsim-web-ui`）；Agent 规划真源在 PluginHost

## 3. 关键类型

- `AiAssistantDockWidget` / `AiAssistantCoordinator`
- `AiConfirmPanel` / `AiLlmSettingsDialog`

## 4. 相关文档

- [CloudSimPluginHost](../CloudSimPluginHost/DEVELOPER_GUIDE.md)
- [CloudSimAiSDK](../../Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md)
