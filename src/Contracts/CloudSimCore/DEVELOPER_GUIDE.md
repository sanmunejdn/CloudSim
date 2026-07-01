
## 变更历史（2026-06）

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