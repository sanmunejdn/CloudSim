# CloudSimCore

契约 DLL：桌面前端与本地引擎之间的稳定 API（仅 Qt Core/Widgets，无 OSG/CGAL/Eigen）。

## 头文件

| 文件 | 说明 |
|------|------|
| `CoreTypes.h` | `ObjectId`、`Mat4`、`PoseDto`、`ColorDto`、`PropertyRowDto`、`ImportOptionsDto`、`PlanResultDto`、`BackendObjectDto`、`GeometryKind`、`FollowSolveContextDto` 等 |
| `CoreEvents.h`（经 `EventHub` 使用） | `SelectionChangedEvent`、`PoseCommittedEvent`、`BackendObjectRegisteredEvent` 等 |
| `IDataService.h` | 对象注册、属性、位姿/颜色、gizmo 写回、Follow、文件导入、单对象 JSON |
| `IRobotService.h` | URDF 注册、FK、plan、程序 JSON |
| `IRenderView.h` | 视口、矩阵、显隐、拾取、`focusCameraOnBackend`、场景快照、选中位姿、gizmo 提交 |
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
| 原始 Data / OSG 场景核心 | `Data.dll`、`OsgWidgetCore.dll`（**Widget 不链接**；经 Host 运行时加载） |

`IRobotService` 由 Host **`RobotServiceAdapter`** 实现（URDF、FK、程序 JSON、基础规划）。

**Host 收口进度**（详见 `ARCHITECTURE_SUMMARY.md` §迁移路线图）：
- 阶段 1.1-1.5 已完成：运动学、规划、程序 JSON、坐标系、TCP IK 通过 `IRobotDocumentHost` 委托
- 阶段 1.6 待定：导出功能需 Controller 内部状态
- **Widget Host-only**：`Widget.dll` 仅链 `CloudSimCore` + `CloudSimHost` + `RobotWidget` + `AiWidget`；对象查询与 Follow 经本契约 DTO/API

**架构说明**：新方法（坐标系捕获、IK 求解等）位于 `IRobotDocumentHost`（RobotWidget），而非 `IRobotService`（CloudSimCore），因为 RobotWidget 不链接 CloudSimCore。

机器人 UI 编排仍在 `RobotWidget` + `DocumentPage`。

---

## DTO 要点（`CoreTypes.h`）

| 类型 | 用途 |
|------|------|
| `PoseDto` | `positionMm` + `eulerDeg`；gizmo / 属性面板读写 |
| `ColorDto` | `r/g/b/a`；`IDataService::applyColor` |
| `BackendObjectDto` | Widget 侧对象快照（id、name、className、parent/child ids、geometryKind、bbox）；**不含** `BackendDataBase*` |
| `GeometryKind` | `None` / `Points` / `Mesh`；替代 Widget 内 `dynamic_cast` Data 类型 |
| `FollowSolveContextDto` | Follow 求解 UI 策略：`skipAll`、`gizmoSelectedBackendId`、`manualPoseAuthorityBackendId` |

---

## `IDataService`（`DataServiceAdapter`）

### 基础查询

| API | Host 行为摘要 |
|-----|----------------|
| `isValid(id)` | 对象是否存在 |
| `clear()` | 清空所有对象 |
| `findByName(name)` | 按名称查找对象 id |
| `className(id)` | 获取对象类名 |
| `displayName(id)` | 获取对象显示名 |
| `boundingBox(id)` | 获取对象包围盒 |

### 对象注册与层级

| API | Host 行为摘要 |
|-----|----------------|
| `registerObject` | `BackendRegistry` 创建实例并 `registerData` |
| `unregisterSubtree` | `DocumentHost::removeBackendSubtree` + `BackendObjectRemovedEvent` |
| `listChildren(parentId)` | 获取子对象 id 列表 |
| `attachChild(parentId, childId)` | 建立父子关系 |

### 属性与 I/O

| API | Host 行为摘要 |
|-----|----------------|
| `propertyRows` | `BackendDataBase::snapshotPropertyRows` → DTO |
| `applyPropertyChange` | 写 Data 后 **`BackendVisualSync`**：mesh/点云 OSG 同步；相关 key 发布 **`PoseCommittedEvent`** |
| `applyWorldPoseMm` | gizmo / 属性面板写回世界位姿（pose 相对几何中心 + 欧拉角）；内部走 `afterDataServicePropertyChange` |
| `applyColor` | 写颜色 + OSG 同步 |
| `worldPoseMm` | 读 Data pose/rotation → `PoseDto` |
| `importFromFile` | `DocumentImportFacade::importFileIntoDocument`；`isPointCloud == true` 时点云，否则 mesh（obj/stl/ply/off + dxf/step/层级）。**透明行为**：`.ply` 且 `PlyIo::plyFileHasTriangleFaces` 时 Host 内部改 mesh（`Model`），插件无需单独分支 |
| `loadObjectFromJson` / `saveObjectToJson` | 工程对象级 JSON（`BackendProjectObjectIo`） |

### 对象快照（Widget 不持有 `BackendDataBase`）

| API | 说明 |
|-----|------|
| `objectSnapshot(id)` | 单对象 `BackendObjectDto` |
| `listObjectSnapshots()` | 全表快照 |
| `geometryKind(id)` | 点云 / 网格 / 无几何 |
| `hasComponent(id, type)` | 是否含指定组件（如 `FollowAttachment`） |

### 树构建支持

| API | 说明 |
|-----|------|
| `topoOrder()` | 返回拓扑排序后的对象 id 列表 |
| `listAll()` | 返回所有对象 id 列表 |
| `parentsOf(id)` | 返回指定对象的父对象 id 列表 |

### Follow（经契约，Widget 不直接调 Host Follow 求解器）

| API | Host 行为摘要 |
|-----|----------------|
| `applyFollowTargetByName(followerId, targetName)` | 等价 `applyPropertyChange(..., "follow.targetName", ...)` |
| `markFollowDirtyFromMove(seedId)` | 置跟随脏集 |
| `requestFollowSolveForced()` | 下一帧强制整图 Follow 求解 |
| `runFollowSolveAndSync(ctx)` | `FollowSolveContextDto` → Host `runBackendFollowSolveAndSync` |

**未在契约内、由 Host 头文件导出**：`DocumentImportFacade::registerAdoptedMesh` / `registerAdoptedPointCloud`、`PointCloudBackgroundLoadState`（已构造几何 + OSG / 后台 ply Job，供 AI/插件/Widget Job 完成回调）。

---

## `IRenderView`（`OsgRenderViewAdapter`）

### 基础操作

| API | 说明 |
|-----|------|
| `widget()` | 底层 `QWidget*`（运行时为 `OsgWidget`） |
| `requestRedraw()` | 请求重绘 |
| `setPickHandler(handler)` / `clearPickHandler()` | 拾取回调管理 |

### 矩阵与显隐

| API | 说明 |
|-----|------|
| `setWorldMatrix` / `getWorldMatrix` | 列主序 `Mat4` ↔ OSG |
| `setVisible` / `removeVisual` / `hasVisualBranch` | 场景分支显隐与存在性 |
| `tryGetModelCenterMm` | 获取模型中心坐标（mm） |

### 相机与层级

| API | 说明 |
|-----|------|
| `focusCameraOnBackend` | 逻辑子树聚合包围球对焦 |
| `setBackendLogicalParent` | 仅 Data 旁路父 id（DXF/STEP 分件） |

### 场景快照与选中查询

| API | 说明 |
|-----|------|
| `sceneGraphSnapshot(maxDepth)` | 返回场景树 DTO（`SceneNodeInfo`），替代 Widget 直接遍历 OSG |
| `selectedPosition(outX, outY, outZ)` | 获取选中对象位置 |
| `selectedRotationEulerDeg(outRx, outRy, outRz)` | 获取选中对象旋转（度） |

### 选中加载与 gizmo 写回（Widget Host-only 新增）

| API | 说明 |
|-----|------|
| `ensureSelectionVisualForBackend(id, urdfLinkMesh)` | 树/OSG 选中时加载或同步几何分支 |
| `syncOuterPatFromBackend(id)` | 从 backend pose 写 outer 世界矩阵 |
| `geometryKindForBackend(id)` | 与 `IDataService::geometryKind` 一致，供选择服务分支 |
| `commitGizmoPoseToBackend(id)` | 罗盘松手：OSG 选中态写回 Data + 发布 `PoseCommitted` |

**`SceneNodeInfo` 结构体**：
```cpp
struct SceneNodeInfo {
    QString className;
    QString name;
    QString localMatrixSummary;
    std::vector<SceneNodeInfo> children;
};
```

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

更细的选中加载/显隐批量操作仍在 `BackendSceneDocumentFacade`（Host 编译，非 Core 契约）。

**计划新增方法**（阶段 4，减少 Widget 对 `OsgWidget*` 具体类型依赖）：

| 方法 | 说明 |
|------|------|
| `setInstructionPoseAxes(...)` | 运动点坐标轴显示 |
| `setRobotFrameOverlays(...)` | 工具/用户坐标系叠加 |
| `beginTcpDragTeach` / `endTcpDragTeach` | TCP 示教模式 |
| `setRawTrajectoryOverlay(...)` / `clearRawTrajectoryOverlay` | 原始轨迹预览 |

---

## `EventHub` 事件（`CoreEvents.h`）

| 事件 | 典型发布方 | 典型订阅方 |
|------|------------|------------|
| `BackendObjectRegisteredEvent` | Host 注册/导入 | `MainWindow` 刷新后端树 |
| `BackendObjectRemovedEvent` | `removeBackendSubtree` | `MainWindow` 刷新后端树 |
| `SelectionChangedEvent` | `MainWindowSelectionService`（树选中后） | `MainWindowUiSetup` → `updatePropertyPanel` |
| `PoseCommittedEvent` | gizmo 松手、`BackendVisualSync`、`publishPoseCommittedFromBackendId` | `MainWindowUiSetup` → `updatePropertyPanel` |
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

- **`Widget.dll`**：链接 **`CloudSimCore.lib` + `CloudSimHost.lib`** + `RobotWidget.lib` + `AiWidget.lib`；**不**链 `Data.lib`；`DocumentPage` 继承 `cloudsim::host::DocumentHost`，对外仅用 `data()` / `robot()` / `render()`。
- **`CloudSim.exe`**：`cloudsimSetApplicationContext(cloudsimCreateApplicationContext())`；`MainWindow` 使用 `cloudsimApplicationContext()->events()`。
- **`CloudSimPluginHost`**（编进 **Host**）：见 [`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)。

## 相关文档

| 文档 | 内容 |
|------|------|
| [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) | Host 适配器、导入、工程包、Follow、`BackendVisualSync` |
| [`Widget/DEVELOPER_GUIDE.md`](../../UI/Widget/DEVELOPER_GUIDE.md) | 主窗口、选择、属性面板、I/O |
| [`ARCHITECTURE_SUMMARY.md`](../../../ARCHITECTURE_SUMMARY.md) | 全局架构 |
