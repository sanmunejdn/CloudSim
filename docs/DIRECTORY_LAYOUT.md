# CloudSim 目录布局

解决方案入口为 [`CloudSim.sln`](../CloudSim.sln)。源码按功能域集中在 `src/`；`docs/` 与 `.cursor/` 位于 `CloudSim/` 根下。文档入口见 [`README.md`](README.md)。

源码格式（编码、头卫、clang-format、筛选器）见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。

## 当前目录树

```text
CloudSim/
├── CloudSim.sln                 # 桌面端：约 48 个 C++ 工程（按功能域解决方案文件夹）
├── CloudSimWeb.sln              # 网页端：无头宿主 + WebGateway（同上文件夹约定）
├── Directory.Build.props
├── .clang-format
├── docs/                        # 常读 + 专题；历史见 docs/_archive/
├── web/cloudsim-web-ui/         # Vite+React 正式壳 → bin/x64*(d)/web
├── scripts/                     # 格式/筛选器等维护脚本
├── tools/                       # 验证工具、训练、维护脚本
├── .cursor/
└── src/
    ├── App/
    │   ├── CloudSim/              # 可执行入口
    │   └── CloudSimBootstrap/     # 组合根 API 头（实现于 CloudSimHost）
    ├── Contracts/
    │   └── CloudSimCore/          # 前后端契约 DLL
    ├── Host/
    │   └── CloudSimHost/          # 文档宿主；inc|source/osg + pluginhost；Shared.items.props
    ├── UI/
    │   ├── Widget/                # MainWindow 等（OsgWidget 已迁入 Host）
    │   ├── OsgWidgetCore/
    │   ├── BackendVisual/
    │   ├── RobotWidget/
    │   ├── AiWidget/
    │   ├── CloudSimUiAssets/      # UI 静态资源库
    │   └── CloudSimPluginHost/    # 仅 README 跳转；源码在 Host/pluginhost
    ├── Robot/
    │   ├── RobotScene/
    │   ├── TrajectoryAlgorithm/
    │   ├── TrajectoryAlgorithmBuiltins/
    │   ├── RobotKinematics/
    │   └── RobotUrdf/
    ├── Geometry/
    │   ├── GeometryEngine/
    │   ├── GeometryAlgorithm/
    │   ├── CollisionAlgorithm/    # 网格碰撞
    │   ├── PointCloudAlgorithm/   # 静态库，链入 Data
    │   ├── VcgAlgorithms/
    │   ├── InstantMeshesCore/
    │   └── InstantMeshesLib/
    ├── Data/
    │   ├── Data/
    │   └── PropertyCore/          # 仅头文件
    ├── Plugins/
    │   ├── CloudSimPluginSDK/
    │   ├── CloudSimAiSDK/
    │   ├── CloudSimMeshTrajectorySDK/
    │   ├── PointCloudPlugin/
    │   ├── GeometryPlugin/
    │   ├── GeometricModelingPlugin/
    │   ├── LabelingPlugin/
    │   ├── PointNetPlugin/
    │   ├── ProcessFlowPlugin/
    │   ├── IndustrialCameraSDK/
    │   ├── IndustrialCameraPlugin/
    │   ├── PlcCommSDK/
    │   ├── RobotCommSDK/
    │   ├── PlcCommUI/
    │   ├── PlcCommPlugin/
    │   └── HelloAiPlugin/         # 示例（可不在 sln；保留）
    └── Infra/
        ├── RunLogger/
        └── Progress/              # 横切进度总线（UiSink → 桌面进度对话框）
```

**Visual Studio**：`CloudSim.sln`（桌面）与 `CloudSimWeb.sln`（网页）均用**解决方案文件夹**按功能域分组（与 `src/` 域对齐，不移动磁盘路径；含中文名时文件为 UTF-8 BOM）。维护脚本：`scripts/organize-sln-folders.ps1 [-SlnPath ...]`。

| 解决方案文件夹 | 典型工程 |
|----------------|----------|
| 应用 | `CloudSim` / `CloudSimWeb`、`CloudSimBootstrap`、`SelfTestRunner` |
| 契约 | `CloudSimCore`、`KinematicCore` |
| 宿主 | `CloudSimHost`、`CloudSimHostCore`、`CloudSimHostDesktopViewport`、`CloudSimHostHeadless` |
| 界面 | `Widget`、`OsgWidgetCore`、`BackendVisual`、`RobotWidget`、`AiWidget`、`CloudSimUiAssets` |
| 数据 | `Data` |
| 机器人 | `RobotScene`、`RobotKinematics`、`RobotUrdf`、`RobotPathPlanning`、`TrajectoryAlgorithm*` |
| 算法库 | `GeometryEngine`、`GeometryAlgorithm`、`GeometryServices`、`CollisionAlgorithm`、`PointCloudAlgorithm`、`InstantMeshes*`、`VcgAlgorithms` |
| 插件 / SDK | `CloudSimPluginSDK`、各业务 SDK、`PlcCommUI`、`CloudSimControllerSample` |
| 插件 / 功能 | 各 `*Plugin`（主要在桌面 sln） |
| 网页 | `CloudSimWebGateway`（`CloudSimWeb.sln`） |
| 基础设施 | `RunLogger`、`Progress` |

磁盘分组仍见上表；解决方案文件夹仅影响 Solution Explorer 显示。

## 旧路径对照（迁移参考）

| 旧路径 | 新路径 |
|--------|--------|
| `CloudSim/CloudSim/` | `src/App/CloudSim/` |
| （新增） | `src/Host/CloudSimHost/` |
| `Widget/` | `src/UI/Widget/` |
| `OsgWidgetCore/` | `src/UI/OsgWidgetCore/` |
| `BackendVisual/` | `src/UI/BackendVisual/` |
| `RobotWidget/` | `src/UI/RobotWidget/` |
| `AiWidget/` | `src/UI/AiWidget/` |
| `CloudSimPluginHost/` | `src/UI/CloudSimPluginHost/` |
| `RobotScene/`、`RobotKinematics/`、`RobotUrdf/` | `src/Robot/.../` |
| `GeometryEngine/`、`GeometryAlgorithm/`、`PointCloudAlgorithm/` | `src/Geometry/.../` |
| `Data/` | `src/Data/Data/` |
| `PropertyCore/` | `src/Data/PropertyCore/` |
| `CloudSimAiSDK/` | `src/Plugins/CloudSimAiSDK/` |
| `CloudSimPluginSDK/` | `src/Plugins/CloudSimPluginSDK/` |
| `PlcCommSDK/` | `src/Plugins/PlcCommSDK/` |
| `PlcCommUI/` | `src/Plugins/PlcCommUI/` |
| `PlcCommPlugin/` | `src/Plugins/PlcCommPlugin/` |
| `RunLogger/` | `src/Infra/RunLogger/` |
| （新增） | `src/Infra/Progress/` |

## 构建与输出

- 统一目录：[`Directory.Build.props`](../Directory.Build.props) 定义（**不依赖** `$(SolutionDir)`）：
  - `$(CloudSimRepoRoot)` → 仓库根（`CloudSim\` 上一级）
  - `$(CloudSimBinDir)` → `bin/x64d/`（Debug）或 `bin/x64/`（Release）
  - `$(CloudSimIntRoot)` → `bin/x64dmiddle/` 或 `bin/x64middle/`
- 普通工程：`OutDir=$(CloudSimBinDir)`，`IntDir=$(CloudSimIntRoot)<工程名>/`
- 插件输出：`$(CloudSimBinDir)plugins/<plugin.id>/`
- 网页静态根：`$(CloudSimBinDir)web/`（`CloudSimWeb` PostBuild / `npm run build:debug|release`）
- 第三方 SDK：`$(CloudSimRepoRoot)bin/SDK/...`（Include / Lib / 部署拷贝）
  - 可选 **SMLib**（NVIDIA usd-brep v1.0.0）：`bin/SDK/smlib/`，见 [`bin/SDK/smlib/README.md`](../../bin/SDK/smlib/README.md)；`CloudSim/SMLib.props` → `CLOUDSIM_HAS_SMLIB`；冒烟 `src/App/SmlibSmoke`
- **生成顺序建议**：`CloudSimCore` → `Data` 等 → **`CloudSimHost`** → `Widget` → `CloudSim`
- 模块职责与文档入口见 [`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md)
- 网页壳开发见 [`web/cloudsim-web-ui/DEVELOPER_GUIDE.md`](../web/cloudsim-web-ui/DEVELOPER_GUIDE.md)

### x64 运行时 DLL（与 exe 同目录）

除 Qt/OSG/OCCT 等第三方运行时外，产品模块包括：

| 类别 | 文件 |
|------|------|
| 契约/宿主 | `CloudSimCore.dll`、`CloudSimHost.dll` |
| 应用/UI | `CloudSim.exe`、`Widget.dll`、`RobotWidget.dll`、`AiWidget.dll`、`OsgWidgetCore.dll`、`BackendVisual.dll` |
| 数据 | `Data.dll` |
| 几何/机器人 | `GeometryEngine.dll`、`GeometryAlgorithm.dll`、`VcgAlgorithms.dll`、`RobotKinematics.dll`、`RobotUrdf.dll`、`RobotScene.dll` |
| 日志/进度 | `RunLogger.dll`、`Progress.dll` |
| 插件 ABI | `CloudSimPluginSDK.dll`、`CloudSimAiSDK.dll` 等；`plugins/<id>/` 下各插件 DLL |
| PLC | `PlcCommSDK.dll`、`PlcCommUI.dll`、`plctag.dll`（见 `bin/SDK/libplctag-...`） |

`PointCloudAlgorithm`、`TrajectoryAlgorithm(+Builtins)`、`CloudSimUiAssets` 等为**静态库**，不单独作为运行时 DLL 分发。调试工作目录应设为 `bin/x64(d)/`。

## CloudSimPluginHost / OsgWidget 说明

- **物理真源**：`src/Host/CloudSimHost/inc|source/pluginhost/`、`inc|source/osg/`（共享项见 `CloudSimHostShared.items.props`）
- **旧路径**：`src/UI/CloudSimPluginHost/` 仅保留 [README](../src/UI/CloudSimPluginHost/README.md) 跳转；指南见 [`DEVELOPER_GUIDE_PluginHost.md`](../src/Host/CloudSimHost/DEVELOPER_GUIDE_PluginHost.md)
- **禁止**再在 `src/UI/Widget` 维护 OsgWidget 平行副本