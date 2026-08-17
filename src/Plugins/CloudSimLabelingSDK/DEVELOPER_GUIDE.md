# CloudSimLabelingSDK 开发指南

## 定位

`CloudSimLabelingSDK.dll` 提供**无 Qt 依赖**的分割标注会话逻辑：标签缓冲、Undo/Redo、PLY+NPY 导出。视口交互经 `IPluginLabelingHost`（CloudSimPluginSDK v1.16.0+）由宿主实现。

## 消费者

- `LabelingPlugin.dll`（侧栏标注 + 训练 UI）
- 未来其他插件/工具

## 头文件

| 头文件 | 说明 |
|--------|------|
| `LabelingSession.h` | 点云/网格标注会话 |
| `LabelingTypes.h` | 训练任务 POD（供 UI 解析） |
| `labeling_sdk_global.h` | 导出宏 |

## 导出格式

与 [`tools/pointnet-training/README.md`](../../../tools/pointnet-training/README.md) 一致：`dataset.jsonl` + `data/*.ply` + `*_labels.npy`。

## 相关文档

- [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md) — `labelingHost()`
- [`LabelingPlugin`](../LabelingPlugin/) — 交互 UI 与训练 Tab
