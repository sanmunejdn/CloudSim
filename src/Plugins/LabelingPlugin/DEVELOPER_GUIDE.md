# LabelingPlugin 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `bin/*/plugins/com.cloudsim.labeling/` |
| 职责 | 侧栏标注 UI；数据约定在 CloudSimLabelingSDK |

## 2. 边界

- SDK：[CloudSimLabelingSDK](../CloudSimLabelingSDK/DEVELOPER_GUIDE.md)
- 训练工具：`CloudSim/tools/pointnet-training/`
- 插件索引：[docs/features/插件](../../features/插件/)

## 3. 验证

Debug|x64 + Release|x64 编本工程。
