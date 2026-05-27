# CloudSimCore

契约 DLL：桌面前端与本地引擎之间的稳定 API（仅 Qt Core/Widgets，无 OSG/CGAL/Eigen）。

## 头文件

| 文件 | 说明 |
|------|------|
| `CoreTypes.h` | `ObjectId`、`Mat4`、`PoseDto`、`PropertyRowDto`、`ImportOptionsDto`、`PlanResultDto` 等 |
| `CoreEvents.h`（经 `EventHub` 使用） | `SelectionChangedEvent`、`PoseCommittedEvent`、`BackendObjectRegisteredEvent` 等 |
| `IDataService.h` | 对象注册、属性、文件导入、单对象 JSON |
| `IRobotService.h` | URDF 注册、FK、plan、程序 JSON |
| `IRenderView.h` | 视口、矩阵、显隐、拾取、`focusCameraOnBackend`、`setBackendLogicalParent` |
| `EventHub.h` | 类型化 `publish` / `subscribe` |
| `IDocumentScope.h` | 每文档 `data()` / `robot()` / `render()` |
| `ICloudSimContext.h` | 应用级工厂与 `EventHub` |
| `CloudSimCoreFactories.h` | 后端 DLL 导出的 `cloudsimCreate*Service` |

## 实现位置（当前）

| 能力 | 模块 |
|------|------|
| 契约类型与 `EventHub` | `CloudSimCore.dll` |
| `DocumentHost`、`DataServiceAdapter`、`OsgRenderViewAdapter`、`RobotServiceAdapter`、`DocumentImportFacade`、`ProjectPackageIo` 等 | `CloudSimHost.dll` |
| `cloudsimCreateApplicationContext()` | **`CloudSimHost.dll`**（声明在 `CloudSimBootstrap/inc/CloudSimBootstrap.h`） |
| 原始 Data / OSG 场景核心 | `Data.dll`、`OsgWidgetCore.dll` |

`IRobotService` 由 Host **`RobotServiceAdapter`** 实现（URDF、FK、程序 JSON、基础规划）；机器人 UI 编排仍在 `RobotWidget` + `DocumentPage`。

---

## `IDataService`（`DataServiceAdapter`）

| API | Host 行为摘要 |
|-----|----------------|
| `registerObject` | `BackendRegistry` 创建实例并 `registerData` |
| `unregisterSubtree` | `DocumentHost::removeBackendSubtree` + `BackendObjectRemovedEvent` |
| `propertyRows` | `BackendDataBase::snapshotPropertyRows` → DTO |
| `applyPropertyChange` | 写 Data 后 **`BackendVisualSync`**：mesh/点云 OSG 同步；相关 key 发布 **`PoseCommittedEvent`** |
| `importFromFile` | `DocumentImportFacade::importFileIntoDocument`；`isPointCloud == true` 时点云，否则 mesh（obj/stl/ply/off + dxf/step/层级）。**透明行为**：`.ply` 且 `PlyIo::plyFileHasTriangleFaces` 时 Host 内部改 mesh（`Model`），插件无需单独分支 |
| `loadObjectFromJson` / `saveObjectToJson` | 工程对象级 JSON（`BackendProjectObjectIo`） |

**未在契约内、由 Host 头文件导出**：`DocumentImportFacade::registerAdoptedMesh` / `registerAdoptedPointCloud`（已构造几何 + OSG，供 AI/插件/ply Job）。

---

## `IRenderView`（`OsgRenderViewAdapter`）

| API | 说明 |
|-----|------|
| `widget()` | 底层 `QWidget*`（运行时为 `OsgWidget`） |
| `setWorldMatrix` / `getWorldMatrix` | 列主序 `Mat4` ↔ OSG |
| `setVisible` / `removeVisual` / `hasVisualBranch` | 场景分支显隐与存在性 |
| `setPickHandler` | 拾取回调（可转发为 `SelectionChangedEvent`） |
| `focusCameraOnBackend` | 逻辑子树聚合包围球对焦 |
| `setBackendLogicalParent` | 仅 Data 旁路父 id（DXF/STEP 分件） |

**Widget 取用 OSG**（推荐 `Widget/inc/WidgetDocumentAccess.h`）：

```cpp
#include "WidgetDocumentAccess.h"
OsgWidget* osg = widgetOsgFromPage(page);
```

**Host 取用 OSG**（`Host/inc/DocumentHostAccess.h`）：

```cpp
#include "DocumentHostAccess.h"
OsgWidget* osg = osgWidgetFrom(host);
```

两者均经 `IRenderView::widget()` 转换；头文件须包含 `IRenderView.h`（上述辅助头已包含）。

更细的选中加载/显隐批量操作仍在 `BackendSceneDocumentFacade`（Widget/Host 编译，非 Core 契约）。

---

## `EventHub` 事件（`CoreEvents.h`）

| 事件 | 典型发布方 | 典型订阅方 |
|------|------------|------------|
| `BackendObjectRegisteredEvent` | Host 注册/导入 | `MainWindow` 刷新后端树 |
| `BackendObjectRemovedEvent` | `removeBackendSubtree` | `MainWindow` 刷新后端树 |
| `SelectionChangedEvent` | `MainWindowSelectionService`（树选中后） | `MainWindowUiSetup` → `updatePropertyPanel` |
| `PoseCommittedEvent` | gizmo 松手、`BackendVisualSync` | `MainWindowUiSetup` → `updatePropertyPanel` |
| `ProjectLoadedEvent` | 工程加载完成 | （可扩展） |
| `RobotKinematicsAppliedEvent` | `RobotServiceAdapter::applyJointAnglesRad` | （可扩展） |

`SelectionChangedEvent::source`：`Tree` / `OsgPick` / `Programmatic`。

---

## `ImportOptionsDto` 要点

| 字段 | 说明 |
|------|------|
| `isPointCloud` | `importFromFile` 走路由点云（ply/xyz 等） |
| `catalogTypeName` | 写入 `backendSourceType`（如 `Model`、`PointCloud`） |
| `persistedId` | 工程恢复时 rekey 为该 id |
| `resetViewToHome` | 导入后是否 reset 相机（层级导入常 false） |

---

## 消费方

- **`Widget.dll`**：链接 `CloudSimCore.lib` + `CloudSimHost.lib`；`DocumentPage` 继承 `cloudsim::host::DocumentHost`。
- **`CloudSim.exe`**：`cloudsimSetApplicationContext(cloudsimCreateApplicationContext())`；`MainWindow` 使用 `cloudsimApplicationContext()->events()`。
- **`CloudSimPluginHost`**（编进 Widget）：见 [`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)。

## 相关文档

| 文档 | 内容 |
|------|------|
| [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) | Host 适配器、导入、工程包、Follow |
| [`Widget/DEVELOPER_GUIDE.md`](../../UI/Widget/DEVELOPER_GUIDE.md) | 主窗口、选择、属性面板、I/O |
| [`ARCHITECTURE_SUMMARY.md`](../../../ARCHITECTURE_SUMMARY.md) | 全局架构 |
