# BACKEND_CALLSITE_INVENTORY — `backend()` 调用点

> 统计口径：源码中 `.backend()` / `->backend()`（2026-08 扫描）。  
> **Host 内部**访问 Data SSOT 可接受；**UI / RobotWidget / Gateway / PluginHost** 为收口目标。

## 总览

| 层 | 文件数（约） | 策略 |
|----|--------------|------|
| CloudSimHost（内部） | 多 | 保留；经 Adapter 投影到契约 |
| Widget | 少 | 白名单；新代码禁增 |
| RobotWidget | 中 | 优先改 `data()` / Host 窄服务 |
| CloudSimPluginHost | 高（Geometry/PC） | 逐步经 PluginDocument / IDataService |
| WebGateway | 少 | 优先 Headless / `data()` |

## 按文件（计数 = 匹配次数）

### A. 应保留（Host 组合根内部）

| 文件 | 次数 | 备注 |
|------|------|------|
| `BackendFileImport.cpp` | 20 | 注册/父子/rekey |
| `HeadlessTrajectorySession.cpp` | 13 | 几何/世界坐标解析 |
| `BackendProjectObjectIo.cpp` | 13 | 工程 objects/edges |
| `HeadlessRobotContext.cpp` | 4 | FK / pose sink |
| `ProjectPackageIo.cpp` | 3 | 保存根 |
| `BackendFollowSolve.cpp` | 3 | Follow 求解 |
| `BackendHierarchyFollow.cpp` | 3 | 层级绑定 |
| `DataServiceAdapter.cpp` | 1 | 适配器本体 |
| `OsgRenderViewAdapter.cpp` | 1 | 拾取对象 |
| `HeadlessPointCloudBridge.cpp` | 1 | 命名查重 |
| `PerLinkKinematicsHostImpl.cpp` | 1 | 经 accessor |
| `BackendVisualSync.cpp` | 1 | 属性后同步 |
| `DocumentHostEvents.cpp` | 1 | PoseCommitted |
| `DocumentHost.cpp` / `.h` | 定义 | 泄漏出口 |

### B. 收口目标 — Widget

| 文件 | 次数 | 建议上提 / 替代 |
|------|------|-----------------|
| `DocumentPage.h/.cpp` | 覆盖接口 | 实现 `IRobotDocumentHost::backend` 等；长期删穿透 |
| `MainWindowRobotHost.cpp` | 1 | 转发页 `backend()`；随 DocumentPage 收口 |
| `MainWindowFileImport.cpp` | 2 | CustomDevice `applyQ` 需 mgr；可 Host 窄 API |
| `BackendSceneDocumentFacade.*` | 持有引用 | 门面保留 mgr 合理；UI 勿再直取 |

### C. 收口目标 — RobotWidget

| 文件 | 原次数 | 状态 |
|------|--------|------|
| `RobotSimulationController.cpp` | 8 | **部分完成**：`listData`/`getData` → `listObjects`/`findObject`；运动学/collision 仍白名单 `backend()` |
| `FeatureTrajectoryPageWidget.cpp` | 3 | **完成**查询迁移；`resolveWorkpieceShape(..., backend())` 仍需 mgr |
| `MeshTrajectoryPageWidget.cpp` | 3 | **完成**（`listObjects`/`findObject`；toplevel 辅助仍借 `backend()`） |
| `TrajectoryEditPageWidget.cpp` | 2 | **完成** → `listObjects` |
| `TrajectoryEditSession.cpp` | 1 | 保留：`worldMatrix(mgr)` 需 `BackendDataManager*` |
| `TrajectoryGeometryResolverHost.cpp` | 1 | **完成** → `findObject` |
| `MeshTriangleSelectionUtil.cpp` | 1 | **完成** → `findObject` |

### D. 收口目标 — PluginHost（编入 Host DLL）

| 文件 | 原次数 | 状态 |
|------|--------|------|
| `PluginGeometryHostImpl.cpp` | 23 | **完成** `getData`/`listData` → `findObject`/`listObjects`；toplevel 辅助仍借 mgr |
| `DocumentPointCloudOps.cpp` | 12 | **完成** → `findObject` |
| `PluginDocumentAdapter.cpp` | 5 | **完成** → `listObjects` / `data().isValid|displayName|className` |
| `PluginPointCloudHostImpl.cpp` | 2 | **完成** → `data().findByName` / `findObject` |
| `PluginHostContext.cpp` | 2 | 保留：shape/STEP 解析 API 签名需 `BackendDataManager&` |

### E. 收口目标 — Web

| 文件 | 原次数 | 状态 |
|------|--------|------|
| `WebGatewayApi.cpp` | 1 | 未改（frame overlay） |
| `WebGateway.cpp` | 1 | **完成** → `findObject` |

契约补充：`IRobotDocumentHost` / `DocumentHost` 已有 `findObject` / `listObjects`。

## 契约缺口（上提候选）

| API 候选 | 替代现状 | Wave |
|----------|----------|------|
| `IDataService::findByClassName` | `BackendDataManager::findByClass` | **Wave2 已落地** |
| 类型化 `objectAs*` / Host geometry resolve | `getData` + dynamic_cast | Wave3+ |
| CustomDevice applyQ Host 包装 | `MainWindowFileImport` 直调 | 后续 |
| Kinematics placement Host 包装 | Controller / DocumentPage | 与 Controller 切片协同 |

## 白名单规则（Widget DEVELOPER_GUIDE）

- 允许：运动学写位姿、可见性写回、mesh 几何（存量）
- 禁止：新代码 `#include "BackendDataManager.h"` 与新增 `backend()` 调用
- 拓扑/跟随/属性：`doc->data()`
