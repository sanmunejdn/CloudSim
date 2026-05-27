# CloudSim 目录布局

解决方案入口仍为仓库根下的 [`CloudSim.sln`](../CloudSim.sln)。源码按功能域集中在 `src/` 下；`docs/`、`.cursor/` 与 `ARCHITECTURE_SUMMARY.md` 保持在 `CloudSim/` 根。

## 当前目录树

```text
CloudSim/
├── CloudSim.sln
├── ARCHITECTURE_SUMMARY.md
├── docs/
├── .cursor/
└── src/
    ├── Contracts/CloudSimCore/       # 前后端契约 DLL（IData/IRobot/IRenderView/EventHub）
    ├── Host/CloudSimHost/            # 文档宿主 + OsgWidget 编译 + 组合根实现（见 DEVELOPER_GUIDE.md）
    ├── App/
    │   ├── CloudSim/                 # 可执行入口
    │   └── CloudSimBootstrap/        # 组合根 API 头文件（实现于 CloudSimHost.dll）
    ├── UI/
    │   ├── Widget/
    │   ├── OsgWidgetCore/
    │   ├── BackendVisual/
    │   ├── RobotWidget/
    │   ├── AiWidget/
    │   └── CloudSimPluginHost/       # 独立工程；产品仍编译进 Widget.dll
    ├── Robot/
    │   ├── RobotScene/
    │   ├── TrajectoryAlgorithm/        # 轨迹块框架静态库（ITrajectoryOp、Registry、Codec）
    │   ├── TrajectoryAlgorithmBuiltins/ # 内置 Translate/Rotate/Delete/Duplicate 等
    │   ├── RobotKinematics/
    │   └── RobotUrdf/
    ├── Geometry/
    │   ├── GeometryEngine/
    │   └── PointCloudAlgorithm/
    ├── Data/
    │   ├── Data/
    │   └── PropertyCore/             # 仅头文件
    ├── Plugins/CloudSimAiSDK/
    ├── Plugins/
    │   ├── CloudSimPluginSDK/
    │   └── HelloPlugin/
    └── Infra/RunLogger/
```

**Visual Studio**：`CloudSim.sln` 列出 18 个 `.vcxproj`（含 **CloudSimHost**；无解决方案文件夹），避免 VS2019 将 `src` 等目录误判为「不兼容」工程。IDE 中按工程名浏览；磁盘分组见上表。请打开 [`CloudSim/CloudSim.sln`](../CloudSim.sln)。

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
| `GeometryEngine/`、`PointCloudAlgorithm/` | `src/Geometry/.../` |
| `Data/` | `src/Data/Data/` |
| `PropertyCore/` | `src/Data/PropertyCore/` |
| `CloudSimAiSDK/` | `src/Plugins/CloudSimAiSDK/` |
| `CloudSimPluginSDK/` | `src/Plugins/CloudSimPluginSDK/` |
| `Plugins/HelloPlugin/` | `src/Plugins/HelloPlugin/` |
| `RunLogger/` | `src/Infra/RunLogger/` |

## 构建与输出

- 统一目录：[`Directory.Build.props`](../Directory.Build.props) 定义 `$(CloudSimBinDir)` → 仓库根 `bin/x64d/`（Debug）或 `bin/x64/`（Release），不依赖 `$(SolutionDir)`。
- 插件示例输出：`bin/x64(d)/plugins/com.cloudsim.hello/`。
- **生成顺序建议**：`CloudSimCore` → `Data` 等引擎 → **`CloudSimHost`**（产出 `CloudSimHost.lib`）→ `Widget` → `CloudSim`。

### x64 运行时 DLL（与 exe 同目录）

除 Qt/OSG/OCCT 等第三方运行时外，产品模块 DLL 包括：

| 类别 | 文件 |
|------|------|
| 契约/宿主 | `CloudSimCore.dll`、`CloudSimHost.dll`（含 `DocumentHost`、`OsgWidget`、组合根） |
| 应用/UI | `CloudSim.exe`、`Widget.dll`、`RobotWidget.dll`、`AiWidget.dll`、`CloudSimAiSDK.dll` |
| 数据 | `Data.dll` |
| 共享引擎 | `RunLogger.dll`、`GeometryEngine.dll`、`RobotKinematics.dll`、`RobotUrdf.dll`、`RobotScene.dll`、`BackendVisual.dll`、`OsgWidgetCore.dll` |
| 插件 ABI | `CloudSimPluginSDK.dll`；`plugins/<id>/` 下各插件 DLL |

`PointCloudAlgorithm` **无**独立 DLL（静态链入 `Data.dll`）。调试时工作目录应设为 `bin/x64(d)/`，以便 Windows 加载器解析上述 DLL。

## CloudSimPluginHost 说明

- **sln 工程**：`src/UI/CloudSimPluginHost/`（可选单独编译静态库）。
- **产品路径**：`Widget.vcxproj` 仍 `ClCompile` 引用同目录源码，链接进 `Widget.dll`，避免与独立 Host 工程重复链入 exe。
