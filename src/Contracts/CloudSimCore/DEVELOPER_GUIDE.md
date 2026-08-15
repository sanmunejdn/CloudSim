# CloudSimCore 模块开发文档

## 1. 模块定位

`CloudSimCore` 提供跨 DLL 的契约：文档作用域、数据/渲染/机器人服务接口、DTO、事件总线。不含 Qt 控件实现与 Data/OSG 真源。

## 2. 显示树契约（Units 投影）

Units 后端对象树是 **UI 显示投影**，不是 Data DAG 的完整镜像，也不是 OSG 场景图。

| 概念 | 契约侧 | 显示约定 |
|------|--------|----------|
| 文档 | `IDocumentScope::documentId()` | 每个打开文档 → Units 一个文档根 |
| 对象列表 | `IDataService::listObjectSnapshots()` / `BackendObjectDto` | 每个对象恰好一个节点 |
| 父子 | `BackendObjectDto::parentIds` / `childIds` | **主父投影**：空父 → 挂文档根；否则 `parentIds.front()`；不建次父 `(ref)` 节点 |
| 生命周期事件 | `BackendObjectRegisteredEvent` / `Removed`（含 `documentId`） | 刷新须 **文档作用域**，禁止因单文档变更重建全部打开文档的树 |
| 可见性 | `BackendObjectDto::visible` / `setVisible` | 树勾选为派生视图；写回带正确文档的 `IDataService` |

**不等于**

- OSG 场景调试树（仅活动文档渲染图）
- Data 多父边全集（次父仅真源保留，见 Data 指南「Units 显示投影」）

目标框架（实现见专题）：DisplayForest + DocumentScopedBinder。  
专题：[`../../../docs/_archive/后端对象显示树/`](../../../docs/_archive/后端对象显示树/)。

### 树构建相关 API（`IDataService`）

| API | 用途 |
|-----|------|
| `topoOrder` / `listAll` / `parentsOf` | 层级查询 |
| `objectSnapshot` / `listObjectSnapshots` | Units / 属性快照 |
| `listChildren` / `attachChild` | 边操作（真源经 Host 适配） |

## 3. 变更历史（2026-06）

### IRenderView 扩展
- 新增 `resolvePickScopeBackendId(ObjectId) const`
- 新增 `backendSkipsInnerModelCenterRebase(ObjectId) const`
- 新增 `activeBackendId() const`
- 新增 `setRobotObjectGizmoSyncHook(std::function<bool()>)`
- 新增 `setRobotObjectGizmoFkRefreshHook(std::function<void()>)`

实现：`OsgRenderViewAdapter` 委托 `OsgWidget`。

### 屏幕交互（HiDPI）
- gizmo / TCP 拖动、屏幕投影拾取：`OsgWidgetCore` 使用 Qt **逻辑像素**；OSG `WINDOW` 拾取经 `logicalMouseToPickWindowCoords`（设备像素）。详见 [`OsgWidgetCore/DEVELOPER_GUIDE.md`](../../UI/OsgWidgetCore/DEVELOPER_GUIDE.md) §5.1。

### CoreTypes 新增 DTO
- `RobotPerLinkKinematicsSliceDto`：per-link FK 切片 DTO（`Mat4` 版本）

## 4. 相关文档

- Data SSOT：[`../../Data/Data/DEVELOPER_GUIDE.md`](../../Data/Data/DEVELOPER_GUIDE.md)
- Widget Units：[`../../UI/Widget/DEVELOPER_GUIDE.md`](../../UI/Widget/DEVELOPER_GUIDE.md)
- 显示树专题：[`../../../docs/_archive/后端对象显示树/`](../../../docs/_archive/后端对象显示树/)
