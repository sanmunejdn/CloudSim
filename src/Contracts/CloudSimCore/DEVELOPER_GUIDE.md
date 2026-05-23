# CloudSimCore

契约 DLL：桌面前端与本地引擎之间的稳定 API（仅 Qt Core/Widgets，无 OSG/CGAL/Eigen）。

## 头文件

| 文件 | 说明 |
|------|------|
| `CoreTypes.h` | `ObjectId`、`Mat4`、`PoseDto`、`PropertyRowDto`、`PlanResultDto` 等 |
| `IDataService.h` | 对象注册、属性、导入、单对象 JSON |
| `IRobotService.h` | URDF 注册、FK、plan、程序 JSON |
| `IRenderView.h` | 视口、矩阵、显隐、拾取回调 |
| `EventHub.h` | 类型化 `publish` / `subscribe` |
| `IDocumentScope.h` | 每文档 `data()` / `robot()` / `render()` |
| `ICloudSimContext.h` | 应用级工厂与 `EventHub` |
| `CloudSimCoreFactories.h` | 后端 DLL 导出的 `cloudsimCreate*Service` |

## 实现位置（当前）

| 能力 | 模块 |
|------|------|
| 契约类型与 `EventHub` | `CloudSimCore.dll` |
| `DocumentHost`、`IDataService`/`IRenderView` 适配器、`OsgWidget` 编译 | `CloudSimHost.dll` |
| `cloudsimCreateApplicationContext()` | **`CloudSimHost.dll`**（声明在 `CloudSimBootstrap/inc/CloudSimBootstrap.h`） |
| 原始 Data / OSG 场景核心 | `Data.dll`、`OsgWidgetCore.dll` |

`IRobotService` 在 Host 内为 **占位适配器**；URDF/规划/程序 JSON 仍主要由 `RobotWidget` + `DocumentPage` 转发，待迁入。

## 消费方

- `Widget.dll`：链接 `CloudSimCore.lib` + `CloudSimHost.lib`；`DocumentPage` 继承 `cloudsim::host::DocumentHost`。
- `CloudSim.exe`：启动时 `cloudsimSetApplicationContext(cloudsimCreateApplicationContext())`，`MainWindow` 使用 `cloudsimApplicationContext()->events()`。

宿主实现细节见 [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md)。
