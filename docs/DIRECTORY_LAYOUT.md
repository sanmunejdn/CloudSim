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
    ├── App/CloudSim/                 # 可执行入口
    ├── UI/
    │   ├── Widget/
    │   ├── OsgWidgetCore/
    │   ├── BackendVisual/
    │   ├── RobotWidget/
    │   ├── AiWidget/
    │   └── CloudSimPluginHost/       # 独立工程；产品仍编译进 Widget.dll
    ├── Robot/
    │   ├── RobotScene/
    │   ├── RobotKinematics/
    │   └── RobotUrdf/
    ├── Geometry/
    │   ├── GeometryEngine/
    │   └── PointCloudAlgorithm/
    ├── Data/
    │   ├── Data/
    │   └── PropertyCore/             # 仅头文件
    ├── AI/AiBackend/
    ├── Plugins/
    │   ├── CloudSimPluginSDK/
    │   └── HelloPlugin/
    └── Infra/RunLogger/
```

**Visual Studio**：`CloudSim.sln` 仅列出 17 个 `.vcxproj`（无解决方案文件夹），避免 VS2019 将 `src` 等目录误判为「不兼容」工程。IDE 中按工程名浏览；磁盘分组见上表。请打开 [`CloudSim/CloudSim.sln`](../CloudSim.sln)。

## 旧路径对照（迁移参考）

| 旧路径 | 新路径 |
|--------|--------|
| `CloudSim/CloudSim/` | `src/App/CloudSim/` |
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
| `AiBackend/` | `src/AI/AiBackend/` |
| `CloudSimPluginSDK/` | `src/Plugins/CloudSimPluginSDK/` |
| `Plugins/HelloPlugin/` | `src/Plugins/HelloPlugin/` |
| `RunLogger/` | `src/Infra/RunLogger/` |

## 构建与输出

- `$(SolutionDir)../bin/x64(d)/` 未改；产物仍在 CGAL 工作区 `bin/` 下。
- 插件示例输出：`bin/x64(d)/plugins/com.cloudsim.hello/`。

## CloudSimPluginHost 说明

- **sln 工程**：`src/UI/CloudSimPluginHost/`（可选单独编译静态库）。
- **产品路径**：`Widget.vcxproj` 仍 `ClCompile` 引用同目录源码，链接进 `Widget.dll`，避免与独立 Host 工程重复链入 exe。
