# CloudSimLabelingSDK 开发指南

> **文档导航**：[全库入口](../../README.md) · [全量目录](../../README.md) · [开发手册](../../开发手册/01-总览.md) · [产品索引](../README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

## 定位

`CloudSimLabelingSDK.dll` 是**宿主内部**标注会话引擎（`LabelingSession`），由 `PluginLabelingHostImpl` 消费。插件面 ABI 在 CloudSimPluginSDK（`IPluginLabelingHost` / `PluginLabelingTypes`）；训练 UI POD 亦在 PluginSDK。

## 消费者

- `CloudSimHost`（`PluginLabelingHostImpl`）
- **不是** LabelingPlugin 的链接依赖（插件已去 vestigial 链接）

## 头文件

| 头文件 | 说明 |
|--------|------|
| `LabelingSession.h` | 点云/网格标注会话 |
| `LabelingTypes.h` | 会话侧 POD（训练 POD 见 PluginSDK） |
| `labeling_sdk_global.h` | 导出宏 |

## 导出格式

与 [`tools/pointnet-training/README.md`](../../../tools/pointnet-training/README.md) 一致：`dataset.jsonl` + `data/*.ply` + `*_labels.npy`。

## 相关文档

- [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md) — `labelingHost()`
- [`LabelingPlugin`](../LabelingPlugin/) — 交互 UI 与训练 Tab
