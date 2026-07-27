# CloudSim 目录布局

解决方案入口为 [`CloudSim.sln`](../CloudSim.sln)。源码按功能域集中在 `src/`；`docs/`、`.cursor/` 与 `ARCHITECTURE_SUMMARY.md` 位于 `CloudSim/` 根下。

源码格式（编码、头卫、clang-format、筛选器）见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。

## 当前目录树

```text
CloudSim/
├── CloudSim.sln                 # 约 40 个产品 C++ 工程（无解决方案文件夹）
├── Directory.Build.props
├── ARCHITECTURE_SUMMARY.md
├── .clang-format
├── docs/                        # 常读 + CloudSim全阶段整治；历史见 docs/_archive/
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
    │   └── CloudSimHost/          # 文档宿主、OsgWidget 编译、组合根
    ├── UI/
    │   ├── Widget/
    │   ├── OsgWidgetCore/
    │   ├── BackendVisual/
    │   ├── RobotWidget/
    │   ├── AiWidget/
    │   ├── CloudSimUiAssets/      # UI 静态资源库
    │   └── CloudSimPluginHost/    # 独立工程参考；产品路径编入 Host
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
    │   ├── CloudSimLabelingSDK/
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
        └── RunLogger/
```

**Visual Studio**：`CloudSim.sln` 列出产品 `.vcxproj`（约 **40** 项；**无**解决方案文件夹），避免 VS2019 将 `src` 等目录误判为「不兼容」工程。IDE 中按工程名浏览；磁盘分组见上表。

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

## 构建与输出

- 统一目录：[`Directory.Build.props`](../Directory.Build.props) 定义（**不依赖** `$(SolutionDir)`）：
  - `$(CloudSimRepoRoot)` → 仓库根（`CloudSim\` 上一级）
  - `$(CloudSimBinDir)` → `bin/x64d/`（Debug）或 `bin/x64/`（Release）
  - `$(CloudSimIntRoot)` → `bin/x64dmiddle/` 或 `bin/x64middle/`
- 普通工程：`OutDir=$(CloudSimBinDir)`，`IntDir=$(CloudSimIntRoot)<工程名>/`
- 插件输出：`$(CloudSimBinDir)plugins/<plugin.id>/`
- 第三方 SDK：`$(CloudSimRepoRoot)bin/SDK/...`（Include / Lib / 部署拷贝）
- **生成顺序建议**：`CloudSimCore` → `Data` 等 → **`CloudSimHost`** → `Widget` → `CloudSim`
- 细节见 [`ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §9

### x64 运行时 DLL（与 exe 同目录）

除 Qt/OSG/OCCT 等第三方运行时外，产品模块包括：

| 类别 | 文件 |
|------|------|
| 契约/宿主 | `CloudSimCore.dll`、`CloudSimHost.dll` |
| 应用/UI | `CloudSim.exe`、`Widget.dll`、`RobotWidget.dll`、`AiWidget.dll`、`OsgWidgetCore.dll`、`BackendVisual.dll` |
| 数据 | `Data.dll` |
| 几何/机器人 | `GeometryEngine.dll`、`GeometryAlgorithm.dll`、`VcgAlgorithms.dll`、`RobotKinematics.dll`、`RobotUrdf.dll`、`RobotScene.dll` |
| 日志 | `RunLogger.dll` |
| 插件 ABI | `CloudSimPluginSDK.dll`、`CloudSimAiSDK.dll` 等；`plugins/<id>/` 下各插件 DLL |
| PLC | `PlcCommSDK.dll`、`PlcCommUI.dll`、`plctag.dll`（见 `bin/SDK/libplctag-...`） |

`PointCloudAlgorithm`、`TrajectoryAlgorithm(+Builtins)`、`CloudSimUiAssets` 等为**静态库**，不单独作为运行时 DLL 分发。调试工作目录应设为 `bin/x64(d)/`。

## CloudSimPluginHost 说明

- **sln 工程**：`src/UI/CloudSimPluginHost/`（可选单独编译参考）
- **产品路径**：源码由 **`CloudSimHost.vcxproj` 编入 `CloudSimHost.dll`**（勿再编入 Widget）；细节见 [`ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §10–§11.1
- **OsgWidget 真源**：`src/UI/Widget/source/OsgWidget*`（由 Host 编译）；禁止在 `Host/inc|source/osg` 维护平行副本