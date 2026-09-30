# LabelingPlugin 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `bin/*/plugins/com.cloudsim.labeling/` |
| 职责 | 侧栏标注 UI；训练 POD 单源在 PluginSDK |

## 2. 边界

- 类型：`PluginLabelingTypes.h`（`PluginTrainingEpochMetrics` / `PluginTrainingJobResult`）
- `LabelingSession`：HostCore 内部引擎（`inc/pluginhost/LabelingSession.h`）；**CloudSimLabelingSDK 已移除**，插件勿直链会话实现
- 训练工具：`CloudSim/tools/pointnet-training/`
- 插件索引：[docs/features/插件](../../features/插件/)

## 3. 验证

Debug|x64 + Release|x64 编本工程。
