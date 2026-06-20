# CloudSim 子模块开发文档索??

本文档列出各 Visual Studio 子工程（子模块）??**DEVELOPER_GUIDE.md**。总架构与业务流程??[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)。目录说明见 [`DIRECTORY_LAYOUT.md`](DIRECTORY_LAYOUT.md)??

---

## 统一世界坐标契约（必读）

全工程空间语义以 **[`spatial_contract_world_pose.md`](spatial_contract_world_pose.md) §1.1** 为权威说明。凡涉及导入、显示、FK、配准、拾取、坐标系叠加、TCP 示教??*须先对照该文??*，再改代码??

| 要点 | 约定 |
|------|------|
| **`pose`** | 模型坐标原点在世界中的位??(mm) |
| **`rotation`** | 内禀 **ZYX 内旋**（`R=Rz·Ry·Rx`）；主动旋转：模型系 ??世界??|
| 点变??| `p_world = R×p_model + pose`（列?? `p_model×R + pose`（OSG 行）；桥接见 `Adapters` |
| 几何 | `geometry` ??*世界绝对坐标**；`pose` + `rotation` ??*唯一**刚体偏移 |
| 显示 | outer = `osgMatrixFromRigidTransform`；inner PAT ??`(0,0,0)`；绕原点转时 `pose` 不变 |
| 权威 API | `engine::rigidTransformFromBackendPoseEuler` / `backendPoseEulerFromRigidTransform`（`BackendWorldPose.h`??|
| Gizmo | World/Local 仅交互方式；存盘为总姿??ZYX 分解 |
| URDF per-link | q0 单次 `Tbind` 烘焙顶点 ??`pose=I`；FK：`M = M0·inv(T0)·Tq·P`??*禁止**双重 visual 烘焙 |
| 工具??| 世界烘焙顶点：挂根连??+ `toolTcpInBaseFromFk(该工??`；连杆系顶点：挂法兰 + `T_flange_tool` |
| 已废??| `T(pose)×R` 手写拼装、枢轴补偿改 `pose`、`skipInnerModelCenterRebase` 主路径、质??rebase 配准 |

**强相关模块文??*：[`Data`](../src/Data/Data/DEVELOPER_GUIDE.md) · [`BackendVisual`](../src/UI/BackendVisual/DEVELOPER_GUIDE.md) · [`OsgWidgetCore`](../src/UI/OsgWidgetCore/DEVELOPER_GUIDE.md) · [`CloudSimHost`](../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) · [`RobotUrdf`](../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md) · [`RobotScene`](../src/Robot/RobotScene/DEVELOPER_GUIDE.md) · [`RobotWidget`](../src/UI/RobotWidget/DEVELOPER_GUIDE.md)

| 子工??| 职责简??| 开发文??|
|--------|----------|----------|
| **CloudSim** | 可执行入口、`main` 生命周期 | [CloudSim/DEVELOPER_GUIDE.md](../src/App/CloudSim/DEVELOPER_GUIDE.md) |
| **CloudSimCore** | 前后端契??DLL（DTO、`EventHub`、服务接口） | [CloudSimCore/DEVELOPER_GUIDE.md](../src/Contracts/CloudSimCore/DEVELOPER_GUIDE.md) |
| **CloudSimHost** | 文档宿主、`OsgWidget` 编译、Core 适配器、组合根实现 | [CloudSimHost/DEVELOPER_GUIDE.md](../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) |
| **Widget** | Qt 主窗口、文档页、流程协调；契约??`data()`/`robot()`/`render()`；OsgWidget 信号边界 `WidgetSceneSignalWiring`（OSG 源码??Host 编译??| [Widget/DEVELOPER_GUIDE.md](../src/UI/Widget/DEVELOPER_GUIDE.md) |
| **RobotWidget** | 仿真/设备 Dock UI、`RobotSimulationController`??*CAD 轨迹生成**??*轨迹编辑**、工程机器人 JSON | [RobotWidget/DEVELOPER_GUIDE.md](../src/UI/RobotWidget/DEVELOPER_GUIDE.md) |
| **Data** | 后端对象模型、属性、层级、跟随求解；**`geometry_backend_ops` / `GeometryRef`**；工??v4 序列??| [Data/DEVELOPER_GUIDE.md](../src/Data/Data/DEVELOPER_GUIDE.md) · [backend_persistence/](backend_persistence/) |
| **BackendVisual** | 数据 ??OSG 分支构建策略 | [BackendVisual/DEVELOPER_GUIDE.md](../src/UI/BackendVisual/DEVELOPER_GUIDE.md) |
| **OsgWidgetCore** | ??OSG 场景、拾取、gizmo、绑定索??| [OsgWidgetCore/DEVELOPER_GUIDE.md](../src/UI/OsgWidgetCore/DEVELOPER_GUIDE.md) |
| **GeometryEngine** | `RigidTransform`、`BackendWorldPose`、工具链 FK、`OSG`/`BackendMat4` 适配 | [GeometryEngine/DEVELOPER_GUIDE.md](../src/Geometry/GeometryEngine/DEVELOPER_GUIDE.md) · [CONVENTIONS.md](../src/Geometry/GeometryEngine/CONVENTIONS.md) |
| **GeometryAlgorithm** | OCC/CGAL 离散、求交、布尔；**FeatureSpec**；网格曲面重构；**管状铸件特征构建（环分割??* | [GeometryAlgorithm/DEVELOPER_GUIDE.md](../src/Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.4??.5 |
| **RobotKinematics** | DH 串联 FK / 数??IK | [RobotKinematics/DEVELOPER_GUIDE.md](../src/Robot/RobotKinematics/DEVELOPER_GUIDE.md) |
| **RobotUrdf** | URDF 解析、层级场景??*prismatic FK**、每连杆后端 | [RobotUrdf/DEVELOPER_GUIDE.md](../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md) |
| **RobotScene** | 指令模型、规划、回放??*RawTrajectory**、多程序/分组/轨迹流水??Command | [RobotScene/DEVELOPER_GUIDE.md](../src/Robot/RobotScene/DEVELOPER_GUIDE.md) |
| **TrajectoryAlgorithm** | `ITrajectoryOp` 框架、Registry、Codec、ConfigRegistry | [TrajectoryAlgorithm/DEVELOPER_GUIDE.md](../src/Robot/TrajectoryAlgorithm/DEVELOPER_GUIDE.md) |
| **TrajectoryAlgorithmBuiltins** | 18 种原子块实现、`UnifiedTrajectoryPathMath`、启动注??| [TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md](../src/Robot/TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md) |
| **RunLogger** | 文件/控制??UI 日志（x64 共享 DLL??| [RunLogger/DEVELOPER_GUIDE.md](../src/Infra/RunLogger/DEVELOPER_GUIDE.md) |
| **CloudSimPluginSDK** | 动态插??ABI（宿主上下文、文??场景 API??| [CloudSimPluginSDK/DEVELOPER_GUIDE.md](../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) |
| **CloudSimPluginHost** | 插件扫描、`QPluginLoader`、`PluginHostContext`（编??**`CloudSimHost.dll`**；UI ??`IPluginMainWindowHost`??| [CloudSimPluginHost/DEVELOPER_GUIDE.md](../src/UI/CloudSimPluginHost/DEVELOPER_GUIDE.md) · [ARCHITECTURE_SUMMARY.md §10](../ARCHITECTURE_SUMMARY.md) |
|| **插件开发示例** | 参见 CloudSimPluginSDK 开发指南中的插件模板和示例代码 | [CloudSimPluginSDK/DEVELOPER_GUIDE.md](../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) |
| **CloudSimAiSDK** | AI 助手 ABI、分域专模、`ai_config` 与训练文档索??| [CloudSimAiSDK/DEVELOPER_GUIDE.md](../src/Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md) · [配置](../../tools/ai-training/CONFIGURATION.md) · [训练](../../tools/ai-training/README.md) · [**AI 轨迹特征**](../../docs/trajectory_feature_ai.md) |
| **AiWidget** | AI 助手 Dock、`AiAssistantCoordinator`（含 `trajectory.feature` 会话??| [CloudSimAiSDK/DEVELOPER_GUIDE.md](../src/Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md) · [trajectory_feature_ai.md](../../docs/trajectory_feature_ai.md) |

## 依赖方向（简图）

x64 ??`RunLogger`～`OsgWidgetCore` ??**独立 DLL**（见下表）；箭头为编译期依赖，运行时各消费??`LoadLibrary` 同一??DLL??

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
  RS --> TA[TrajectoryAlgorithm + Builtins 静态]
  RS --> RU --> D
  RS --> RK
  RS --> RL
  RU --> BV
  RU --> GE
  RU --> RL
  D --> RL
  D --> PCA[PointCloudAlgorithm 静态]
  D --> GA[GeometryAlgorithm.dll]
```

## x64 动态库约定

| 工程 | x64 产物 | 构建时定??| 消费者（默认 dllimport??|
|------|---------|-----------|-------------------------|
| RunLogger | `RunLogger.dll` | `RUN_LOGGER_LIB` | ??|
| GeometryEngine | `GeometryEngine.dll` | `GEOMETRY_ENGINE_LIB` | ??|
| RobotKinematics | `RobotKinematics.dll` | `ROBOT_KINEMATICS_LIB` | ??|
| RobotUrdf | `RobotUrdf.dll` | `ROBOT_URDF_LIB` | ??|
| RobotScene | `RobotScene.dll` | `ROBOT_SCENE_LIB` | ??|
| BackendVisual | `BackendVisual.dll` | `BACKENDVISUAL_LIB` | ??|
| OsgWidgetCore | `OsgWidgetCore.dll` | `OSGWIDGETCORE_LIB` | ??|
| Data | `Data.dll` | `DATA_LIB` | ??|
| Widget | `Widget.dll` | `WIDGET_LIB` | ??|
| PointCloudAlgorithm | `.lib`（静态） | `POINT_CLOUD_ALGORITHM_STATIC` | ??`Data` 构建??|
| TrajectoryAlgorithm + Builtins | `.lib`（静态） | `TRAJECTORY_ALGORITHM_LIB` / `ROBOT_SCENE_LIB`（TU??| ??`RobotScene` 链接；UI ??`TrajectoryOpBridge` |

- 导出宏见??`inc/*_global.h`（`RUN_LOGGER_API`、`GEOMETRY_ENGINE_API`、`BACKENDVISUAL_EXPORT`、`OSGWIDGETCORE_EXPORT` 等）??
- **??*在头文件中用 `*_API` 标记 `constexpr` 字符串常量（MSVC dllimport 限制）；改用 `inline constexpr`（见 `RobotInstructionProgram.h` ??`kMotionPointIndexKey`）??
- Win32 遗留配置仍可??`*_STATIC` / `BUILD_STATIC` 关闭导入导出??
- 新增??DLL ??函数：在对应 `*_global.h` 加导出宏，vcxproj 构建侧加 `*_LIB`，消费者通过 `ProjectReference` + import `.lib` 链接??

## Visual Studio 解决方案筛选器

各子工程 `.vcxproj.filters` 统一为两层结构：

1. 顶层??*`inc`**（头文件）??*`src`**（源文件 / Qt Moc??
2. 子层：按功能划分（如 `inc\adapters`、`src\MainWindow`、`src\OsgWidget`、`src\Backend`、`src\ThirdParty\qtpropertybrowser` 等）

跨工程引用（??Widget 编入??`CloudSimPluginHost`、`Host` 引用??Widget OSG 源）使用 `inc\HostRef`、`src\WidgetBorrowed`、`inc\PluginHost` 等筛选器，与本地 `inc`/`src` 区分??

新增/移动源文件后，在 `CloudSim` 目录执行??

```bash
python scripts/generate_vcxproj_filters.py
```

脚本会扫描全??`*.vcxproj` 并重写对应的 `.vcxproj.filters`（不修改 `.vcxproj` 本身）??

## 源码注释约定（code-comment??

- 只写 **Why**：业务背景、非显然算法、边界兜底、危险操作；不写「代码在做什么??
- 头文件公开 API：精简中文 `///`（约 5??5 字）；`@param` 保留但描述用中文
- 实现文件：非显然处用??`//`，行内注释尽???? ??
- 避免「用于…」「该方法…」等翻译腔；行末 `//` 不加句号
- 批量清理（仅??`【中文】` 标签等）：`python scripts/apply_code_comment_style.py`（慎用，改后需 diff 复核??

## 头文件约??

- 公共 API：`各工??inc/*.h`
- 实现：`各工??source/*.cpp`
- ??DLL：使用各 `*_global.h` 中的 `*_EXPORT` 宏；跨域引用优先通过 vcxproj `AdditionalIncludeDirectories`，源码内相对路径需??`src/` 深度书写（见 `DIRECTORY_LAYOUT.md`）??

## 工程持久化（v4??

设计与验收见 [`backend_persistence/`](backend_persistence/)（`DESIGN_*`、`TASK_*`、`REGRESSION_CHECKLIST_*`、`ACCEPTANCE_*`）。架构总览：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §6.5??

## 约定与规??

- C++ / Qt 编码约定：`.cursor/rules/cloudsim-cpp-conventions.mdc`
- 解决方案结构：`.cursor/rules/cloudsim-architecture.mdc`
