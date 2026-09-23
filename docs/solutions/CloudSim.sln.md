# CloudSim.sln（桌面）

入口：`CloudSim/CloudSim.sln` → `CloudSim.exe`。全库入口：[../README.md](../README.md)。

每个工程权威说明在源码旁 `DEVELOPER_GUIDE.md`（或插件 README）。

## App / Contracts / Host / Infra

| 工程 | 产物 | 指南 |
|------|------|------|
| CloudSimBootstrap | 引导相关 | [DEVELOPER_GUIDE](../../src/App/CloudSimBootstrap/DEVELOPER_GUIDE.md) |
| CloudSim | `CloudSim.exe` | [DEVELOPER_GUIDE](../../src/App/CloudSim/DEVELOPER_GUIDE.md) |
| CloudSimCore | 契约 DLL | [DEVELOPER_GUIDE](../../src/Contracts/CloudSimCore/DEVELOPER_GUIDE.md) |
| KinematicCore | 运动学契约 | [DEVELOPER_GUIDE](../../src/Contracts/KinematicCore/DEVELOPER_GUIDE.md) |
| CloudSimHost | 文档宿主 DLL | [DEVELOPER_GUIDE](../../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) |
| RunLogger | 运行日志 | [DEVELOPER_GUIDE](../../src/Infra/RunLogger/DEVELOPER_GUIDE.md) |

## UI

| 工程 | 指南 |
|------|------|
| Widget | [DEVELOPER_GUIDE](../../src/UI/Widget/DEVELOPER_GUIDE.md) |
| RobotWidget | [DEVELOPER_GUIDE](../../src/UI/RobotWidget/DEVELOPER_GUIDE.md) |
| OsgWidgetCore | [DEVELOPER_GUIDE](../../src/UI/OsgWidgetCore/DEVELOPER_GUIDE.md) |
| BackendVisual | [DEVELOPER_GUIDE](../../src/UI/BackendVisual/DEVELOPER_GUIDE.md) |
| AiWidget | [DEVELOPER_GUIDE](../../src/UI/AiWidget/DEVELOPER_GUIDE.md) |
| CloudSimUiAssets | [DEVELOPER_GUIDE](../../src/UI/CloudSimUiAssets/DEVELOPER_GUIDE.md) |

`CloudSimPluginHost` 源码编入 Host，指南：[CloudSimPluginHost](../../src/UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)。

## Data / Robot / Geometry

| 工程 | 指南 |
|------|------|
| Data | [DEVELOPER_GUIDE](../../src/Data/Data/DEVELOPER_GUIDE.md) |
| RobotUrdf / RobotScene / RobotKinematics / RobotPathPlanning | [Urdf](../../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md) · [Scene](../../src/Robot/RobotScene/DEVELOPER_GUIDE.md) · [Kinematics](../../src/Robot/RobotKinematics/DEVELOPER_GUIDE.md) · [PathPlanning](../../src/Robot/RobotPathPlanning/DEVELOPER_GUIDE.md) |
| TrajectoryAlgorithm* | [Trajectory](../../src/Robot/TrajectoryAlgorithm/DEVELOPER_GUIDE.md) · [Builtins](../../src/Robot/TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md) |
| GeometryEngine / GeometryAlgorithm / GeometryServices | [Engine](../../src/Geometry/GeometryEngine/DEVELOPER_GUIDE.md) · [Algorithm](../../src/Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) · [Services](../../src/Geometry/GeometryServices/DEVELOPER_GUIDE.md) |
| Collision / PointCloud / Vcg / InstantMeshes* | [Collision](../../src/Geometry/CollisionAlgorithm/DEVELOPER_GUIDE.md) · [PointCloud](../../src/Geometry/PointCloudAlgorithm/DEVELOPER_GUIDE.md) · [Vcg](../../src/Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md) · [IM Core](../../src/Geometry/InstantMeshesCore/DEVELOPER_GUIDE.md) · [IM Lib](../../src/Geometry/InstantMeshesLib/DEVELOPER_GUIDE.md) |

## Plugins

| 工程 | 指南 |
|------|------|
| CloudSimPluginSDK / AiSDK / LabelingSDK / MeshTrajectorySDK | [PluginSDK](../../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) · [Ai](../../src/Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md) · [Labeling](../../src/Plugins/CloudSimLabelingSDK/DEVELOPER_GUIDE.md) · [MeshTraj](../../src/Plugins/CloudSimMeshTrajectorySDK/DEVELOPER_GUIDE.md) |
| GeometricModeling / ProcessFlow / EngineeringDrawing | [Geomodel](../../src/Plugins/GeometricModelingPlugin/DEVELOPER_GUIDE.md) · [Process](../../src/Plugins/ProcessFlowPlugin/DEVELOPER_GUIDE.md) · [Drawing](../../src/Plugins/EngineeringDrawingPlugin/DEVELOPER_GUIDE.md) |
| Geometry / PointCloud / PointNet / Labeling | [Geometry](../../src/Plugins/GeometryPlugin/DEVELOPER_GUIDE.md) · [PointCloud](../../src/Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md) · [PointNet](../../src/Plugins/PointNetPlugin/DEVELOPER_GUIDE.md) · [LabelingPlugin](../../src/Plugins/LabelingPlugin/DEVELOPER_GUIDE.md) |
| PlcComm* / IndustrialCamera* / RobotCommSDK | [Plc SDK](../../src/Plugins/PlcCommSDK/DEVELOPER_GUIDE.md) · [Plc UI](../../src/Plugins/PlcCommUI/DEVELOPER_GUIDE.md) · [Plc Plugin](../../src/Plugins/PlcCommPlugin/DEVELOPER_GUIDE.md) · [Cam SDK](../../src/Plugins/IndustrialCameraSDK/DEVELOPER_GUIDE.md) · [Cam Plugin](../../src/Plugins/IndustrialCameraPlugin/DEVELOPER_GUIDE.md) · [RobotComm](../../src/Plugins/RobotCommSDK/DEVELOPER_GUIDE.md) |

模式专题：[../features/主程序](../features/主程序/) · [几何建模](../features/几何建模/) · [工艺流程](../features/工艺流程/) · [工程图](../features/工程图/) · [插件](../features/插件/)。
