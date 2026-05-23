# CloudSim 子模块开发文档索引

本文档列出各 Visual Studio 子工程（子模块）的 **DEVELOPER_GUIDE.md**。总架构与业务流程见 [`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)。目录说明见 [`DIRECTORY_LAYOUT.md`](DIRECTORY_LAYOUT.md)。

| 子工程 | 职责简述 | 开发文档 |
|--------|----------|----------|
| **CloudSim** | 可执行入口、`main` 生命周期 | [CloudSim/DEVELOPER_GUIDE.md](../src/App/CloudSim/DEVELOPER_GUIDE.md) |
| **CloudSimCore** | 前后端契约 DLL（DTO、`EventHub`、服务接口） | [CloudSimCore/DEVELOPER_GUIDE.md](../src/Contracts/CloudSimCore/DEVELOPER_GUIDE.md) |
| **CloudSimHost** | 文档宿主、`OsgWidget` 编译、Core 适配器、组合根实现 | [CloudSimHost/DEVELOPER_GUIDE.md](../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) |
| **Widget** | Qt 主窗口、文档页、流程协调（OSG 源码由 Host 编译） | [Widget/DEVELOPER_GUIDE.md](../src/UI/Widget/DEVELOPER_GUIDE.md) |
| **RobotWidget** | 仿真/设备 Dock UI、`RobotSimulationController`、工程机器人 JSON | [RobotWidget/DEVELOPER_GUIDE.md](../src/UI/RobotWidget/DEVELOPER_GUIDE.md) |
| **Data** | 后端对象模型、属性、层级、跟随求解、**工程 v4 序列化** | [Data/DEVELOPER_GUIDE.md](../src/Data/Data/DEVELOPER_GUIDE.md) · [backend_persistence/](backend_persistence/) |
| **BackendVisual** | 数据 → OSG 分支构建策略 | [BackendVisual/DEVELOPER_GUIDE.md](../src/UI/BackendVisual/DEVELOPER_GUIDE.md) |
| **OsgWidgetCore** | 纯 OSG 场景、拾取、gizmo、绑定索引 | [OsgWidgetCore/DEVELOPER_GUIDE.md](../src/UI/OsgWidgetCore/DEVELOPER_GUIDE.md) |
| **GeometryEngine** | `RigidTransform`、工具链 FK、`OSG`/`BackendMat4` 适配 | [GeometryEngine/DEVELOPER_GUIDE.md](../src/Geometry/GeometryEngine/DEVELOPER_GUIDE.md) · [CONVENTIONS.md](../src/Geometry/GeometryEngine/CONVENTIONS.md) |
| **RobotKinematics** | DH 串联 FK / 数值 IK | [RobotKinematics/DEVELOPER_GUIDE.md](../src/Robot/RobotKinematics/DEVELOPER_GUIDE.md) |
| **RobotUrdf** | URDF 解析、层级场景、每连杆后端 | [RobotUrdf/DEVELOPER_GUIDE.md](../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md) |
| **RobotScene** | 指令模型、规划、回放、场景 FK | [RobotScene/DEVELOPER_GUIDE.md](../src/Robot/RobotScene/DEVELOPER_GUIDE.md) |
| **RunLogger** | 文件/控制台/UI 日志（x64 共享 DLL） | [RunLogger/DEVELOPER_GUIDE.md](../src/Infra/RunLogger/DEVELOPER_GUIDE.md) |
| **CloudSimPluginSDK** | 动态插件 ABI（宿主上下文、文档/场景 API） | [CloudSimPluginSDK/DEVELOPER_GUIDE.md](../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) |
| **CloudSimPluginHost** | 插件扫描、`QPluginLoader`、宿主实现（`src/UI/CloudSimPluginHost/`，编译进 `Widget.dll`） | 见 [ARCHITECTURE_SUMMARY.md §10](../ARCHITECTURE_SUMMARY.md) |
| **HelloPlugin** | 官方示例插件（Dock + 菜单 + 创建立方体） | [HelloPlugin/plugin.json](../src/Plugins/HelloPlugin/plugin.json) |

## 依赖方向（简图）

x64 下 `RunLogger`～`OsgWidgetCore` 为 **独立 DLL**（见下表）；箭头为编译期依赖，运行时各消费者 `LoadLibrary` 同一份 DLL。

```mermaid
flowchart TB
  EXE[CloudSim.exe] --> W[Widget.dll]
  EXE --> H[CloudSimHost.dll]
  EXE --> C[CloudSimCore.dll]
  W --> H
  W --> C
  H --> C
  H --> O[OsgWidgetCore.dll]
  H --> BV[BackendVisual.dll]
  H --> D[Data.dll]
  W --> RW[RobotWidget.dll]
  RW --> RS[RobotScene.dll]
  RW --> RU[RobotUrdf.dll]
  RW --> RK[RobotKinematics.dll]
  W --> RS
  W --> RU
  W --> RK
  W --> GE[GeometryEngine.dll]
  W --> RL[RunLogger.dll]
  W --> PH[CloudSimPluginHost 编进 Widget]
  PH --> SDK[CloudSimPluginSDK.dll]
  PLG[plugins/*.dll] --> SDK
  O --> BV
  BV --> D
  O --> RL
  BV --> GE
  RS --> GE
  RS --> RU --> D
  RS --> RK
  RS --> RL
  RU --> BV
  RU --> GE
  RU --> RL
  D --> RL
  D --> PCA[PointCloudAlgorithm 静态]
```

## x64 动态库约定

| 工程 | x64 产物 | 构建时定义 | 消费者（默认 dllimport） |
|------|---------|-----------|-------------------------|
| RunLogger | `RunLogger.dll` | `RUN_LOGGER_LIB` | 无 |
| GeometryEngine | `GeometryEngine.dll` | `GEOMETRY_ENGINE_LIB` | 无 |
| RobotKinematics | `RobotKinematics.dll` | `ROBOT_KINEMATICS_LIB` | 无 |
| RobotUrdf | `RobotUrdf.dll` | `ROBOT_URDF_LIB` | 无 |
| RobotScene | `RobotScene.dll` | `ROBOT_SCENE_LIB` | 无 |
| BackendVisual | `BackendVisual.dll` | `BACKENDVISUAL_LIB` | 无 |
| OsgWidgetCore | `OsgWidgetCore.dll` | `OSGWIDGETCORE_LIB` | 无 |
| Data | `Data.dll` | `DATA_LIB` | 无 |
| Widget | `Widget.dll` | `WIDGET_LIB` | 无 |
| PointCloudAlgorithm | `.lib`（静态） | `POINT_CLOUD_ALGORITHM_STATIC` | 仅 `Data` 构建侧 |

- 导出宏见各 `inc/*_global.h`（`RUN_LOGGER_API`、`GEOMETRY_ENGINE_API`、`BACKENDVISUAL_EXPORT`、`OSGWIDGETCORE_EXPORT` 等）。
- **勿**在头文件中用 `*_API` 标记 `constexpr` 字符串常量（MSVC dllimport 限制）；改用 `inline constexpr`（见 `RobotInstructionProgram.h` 中 `kMotionPointIndexKey`）。
- Win32 遗留配置仍可用 `*_STATIC` / `BUILD_STATIC` 关闭导入导出。
- 新增跨 DLL 类/函数：在对应 `*_global.h` 加导出宏，vcxproj 构建侧加 `*_LIB`，消费者通过 `ProjectReference` + import `.lib` 链接。

## 头文件约定

- 公共 API：`各工程/inc/*.h`
- 实现：`各工程/source/*.cpp`
- 跨 DLL：使用各 `*_global.h` 中的 `*_EXPORT` 宏；跨域引用优先通过 vcxproj `AdditionalIncludeDirectories`，源码内相对路径需按 `src/` 深度书写（见 `DIRECTORY_LAYOUT.md`）。

## 工程持久化（v4）

设计与验收见 [`backend_persistence/`](backend_persistence/)（`DESIGN_*`、`TASK_*`、`REGRESSION_CHECKLIST_*`、`ACCEPTANCE_*`）。架构总览：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §6.5。

## 约定与规则

- C++ / Qt 编码约定：`.cursor/rules/cloudsim-cpp-conventions.mdc`
- 解决方案结构：`.cursor/rules/cloudsim-architecture.mdc`
