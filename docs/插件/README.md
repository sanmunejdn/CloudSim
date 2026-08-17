# 插件索引（按类型）

运行时扫描 `bin/x64(d)/plugins/<id>/plugin.json`。契约见 [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md)。

## 工作区模式插件

互斥占用中央工作区（顶栏分段切换）。

| ID | 工程 | 文档 |
|----|------|------|
| `com.cloudsim.geomodeling` | GeometricModelingPlugin | [几何建模/](../几何建模/)、[插件 README](../../src/Plugins/GeometricModelingPlugin/README.md) |
| `com.cloudsim.processflow` | ProcessFlowPlugin | [工艺流程/](../工艺流程/)、[插件 README](../../src/Plugins/ProcessFlowPlugin/README.md) |
| `com.cloudsim.drawing` | EngineeringDrawingPlugin | [工程图/](../工程图/)、[插件 README](../../src/Plugins/EngineeringDrawingPlugin/README.md) |

## 侧栏 / 工具插件

主程序模式下扩展右侧面板或菜单。

| ID | 工程 | 文档 |
|----|------|------|
| `com.cloudsim.geometry` | GeometryPlugin | [DEVELOPER_GUIDE](../../src/Plugins/GeometryPlugin/DEVELOPER_GUIDE.md) |
| `com.cloudsim.pointcloud` | PointCloudPlugin | [DEVELOPER_GUIDE](../../src/Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md) |
| `com.cloudsim.plccomm` | PlcCommPlugin (+ SDK/UI) | [Plugin](../../src/Plugins/PlcCommPlugin/DEVELOPER_GUIDE.md)、[SDK](../../src/Plugins/PlcCommSDK/DEVELOPER_GUIDE.md)、[UI](../../src/Plugins/PlcCommUI/DEVELOPER_GUIDE.md) |
| `com.cloudsim.industrialcamera` | IndustrialCameraPlugin (+ SDK) | [Plugin](../../src/Plugins/IndustrialCameraPlugin/DEVELOPER_GUIDE.md)、[SDK](../../src/Plugins/IndustrialCameraSDK/DEVELOPER_GUIDE.md) |
| `com.cloudsim.labeling` | LabelingPlugin (+ LabelingSDK) | [README](../../src/Plugins/LabelingPlugin/README.md)、[SDK](../../src/Plugins/CloudSimLabelingSDK/DEVELOPER_GUIDE.md) |
| `com.cloudsim.pointnet` | PointNetPlugin | [DEVELOPER_GUIDE](../../src/Plugins/PointNetPlugin/DEVELOPER_GUIDE.md) |
| `com.cloudsim.helloai` | HelloAiPlugin | [README](../../src/Plugins/HelloAiPlugin/README.md) |

## SDK / 非独立侧栏

| 工程 | 说明 | 文档 |
|------|------|------|
| CloudSimPluginSDK | 插件 ABI | [DEVELOPER_GUIDE](../../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) |
| CloudSimAiSDK / AiWidget | AI 助手与分域 | [DEVELOPER_GUIDE](../../src/Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md) |
| CloudSimMeshTrajectorySDK | Mesh 轨迹会话（RobotWidget 直连） | [DEVELOPER_GUIDE](../../src/Plugins/CloudSimMeshTrajectorySDK/DEVELOPER_GUIDE.md) |
| RobotCommSDK | 机器人通讯 | [DEVELOPER_GUIDE](../../src/Plugins/RobotCommSDK/DEVELOPER_GUIDE.md) |

宿主扫描与上下文：[`CloudSimPluginHost`](../../src/UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)（编入 `CloudSimHost.dll`）。
