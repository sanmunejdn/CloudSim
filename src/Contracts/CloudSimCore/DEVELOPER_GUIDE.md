
## 变更历史（2026-06）

### IRenderView 扩展
- 新增 `resolvePickScopeBackendId(ObjectId) const`
- 新增 `backendSkipsInnerModelCenterRebase(ObjectId) const`
- 新增 `activeBackendId() const`
- 新增 `setRobotObjectGizmoSyncHook(std::function<bool()>)`
- 新增 `setRobotObjectGizmoFkRefreshHook(std::function<void()>)`

实现：`OsgRenderViewAdapter` 委托 `OsgWidget`。

### CoreTypes 新增 DTO
- `RobotPerLinkKinematicsSliceDto`：per-link FK 切片 DTO（`Mat4` 版本）