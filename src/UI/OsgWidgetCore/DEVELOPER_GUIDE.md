# OsgWidgetCore 模块开发文档

## 1. 模块定位

`OsgWidgetCore` 是 **与 Qt 无关** 的 OSG 场景核心：分层场景根、相机导航、后端对象绑定与拾取、点/边/面拾取、注释、gizmo 罗盘、`ObjectGizmoFrame` 位姿数学。重绘通过 `setRequestRedraw` 回调由 `Widget::OsgWidget` 注入。

| 属性 | 说明 |
|------|------|
| 主类 | `OsgScene`（头文件 `OsgScene.h`，实现分 `OsgScene*.cpp`） |
| x64 输出 | `OsgWidgetCore.dll` |
| 依赖 | `BackendVisual.dll`、`Data.dll`、`RunLogger.dll` |
| 导出 | `OSGWIDGETCORE_EXPORT`（x64 构建：`OSGWIDGETCORE_LIB`） |
| 头文件注意 | 导出类若含 `osg::ref_ptr<osgViewer::Viewer>` 等成员，须在头文件中 `#include` 完整 OSG 类型（非仅前向声明） |

---

## 2. 场景分层（`initScene` 后）

```text
m_root
└─ m_sceneContentGroup
   ├─ 注释层
   ├─ m_backendObjectRoots（导入点云/网格 outer PAT）
   ├─ m_robotAssemblyRoot（URDF 层级装配）
   └─ m_trajectoryOverlayRoot（轨迹/调试）
m_gizmoOverlayGroup（罗盘，选中时挂到 active inner PAT 下）
m_stagingGroup（导入预览）
```

| 访问器 | 作用 |
|--------|------|
| `sceneContentRoot()` | 注释 + 后端 + 机器人 + 轨迹 |
| `backendObjectsRoot()` | 后端对象容器 |
| `robotAssemblyRoot()` | 关节 MatrixTransform 树 |
| `trajectoryOverlayRoot()` | 预留覆盖层 |

---

## 3. `class ObjectGizmoFrame`

**语义**：outer 局部矩阵 = **`T(centerPlusPose) * R(attitude)`**（行向量 OSG）；文件原点在 inner 局部 (0,0,0)。inner PAT 平移为 **`-modelCenter`**（`MeshBackendVisual::buildOuterBranch`）。

**枢轴（文件原点）在 outer 父节点下**：`(inner + centerPlusPose) * R`，其中 `inner` 为 inner 在 outer 局部的平移（通常 `-modelCenter`）。**禁止**用 `centerPlusPose + inner*R` 或 `decompose(outer).translation` 当作 `centerPlusPose`：`decompose` 得到的是原点位置 `trans*R`，不是 `trans`。

| 字段 / 方法 | 说明 |
|-------------|------|
| `modelCenter` | 与 `m_backendModelCenters` 一致 |
| `centerPlusPose` | outer 平移分量 `trans`：`modelCenter + backend.pose`（与 `buildOuterBranch` 的 `T(trans)*R` 一致） |
| `attitude` | outer 旋转 `R` |
| `backendPoseRelativeToCenter()` | 即后端 `pose`（`centerPlusPose - modelCenter`） |
| `fromOuter(outer, modelCenter, out)` | 从场景读帧：`trans = (fileInOuterParent - inner*R) * inv(R)` |
| `setFromBackend(poseRelCenter, attitude, modelCenter)` | 从后端写帧 |
| `applyToOuter(outer)` | `setMatrix(T(centerPlusPose)*R)` |
| `translateAlongWorldDirection` | 沿**场景世界**方向移动枢轴（内部 `worldDirectionToOuterParent`） |
| `translateAlongWorldAxis` / `translateAlongBodyAxis` | 沿世界/物体轴索引平移 |
| `dragAxisDirectionSceneWorld` | 环法向 / **屏幕转角**用：经 `parentWorld` 转到场景世界 |
| `dragAxisDirectionOuterParent` | **四元数旋转**用：World 为场景轴投到 outer 父空间；Local 为 `R*localAxis` |
| `adjustCenterPlusPoseForRotationDelta` | 保枢轴：固定 `(inner+trans)*R`，更新 `trans` 与 `R` |
| `rotatePreMultiplyWorldAxis` / `rotatePostMultiplyLocalAxis` | 属性面板等：`δq*R` / `R*δq`（轴在 outer 局部） |
| `setRotationKeepingPivotInOuterParent` | 给定父空间枢轴与 `R_new` 反解 `trans` |
| `pivotWorldFromOuter` / `pivotInOuterParentFromOuter` | 枢轴诊断 |

---

## 4. `class BackendVisualBindingIndex`

| 方法 | 作用 |
|------|------|
| `bindBackendRoot(backendId, rootNode)` | 注册 outer；`observer_ptr` 防悬空 |
| `unbindBackend` / `clear` | 移除 |
| `resolveBackendIdFromNodePath(path, outBackendId)` | 优先索引，回退 `BackendIdUserData` |

---

## 5. `class OsgScene` — 公共 API 分组

### 5.1 生命周期与视口

| 方法 | 说明 |
|------|------|
| `setRequestRedraw(fn)` / `requestRedraw()` | 宿主重绘 |
| `setViewportPixels(w,h)` / `setDevicePixelRatio(dpr)` | 逻辑像素与 HiDPI |

### 5.2 初始化与 HUD

| 方法 | 说明 |
|------|------|
| `initScene()` | 构建整图 |
| `initWorldAxesHud()` / `updateWorldAxesHudViewport` | 角标世界轴 |
| `applyHeadlightToViewer(viewer)` | HEADLIGHT |

### 5.3 后端层级与 Gizmo 同步

| 方法 | 说明 |
|------|------|
| `isBackendDescendantOf(child, ancestor)` | 逻辑父链 `m_backendParentIds` |
| `setBackendLogicalParent(child, parent)` | 仅写 `m_backendParentIds`，**不** reparent OSG（DXF 分件 + `focusCameraOnBackend` 聚合子树） |
| `backendOuterPatIsUnderOuterPatInSceneGraph` | OSG 父链 |
| `syncGizmoAndPickFromBackend(data)` | 选中：无父则 `setFromBackend+applyToOuter`；**有父仅 `fromOuter`** |
| `syncSelectionForBackendId` | 切换 active + 挂 overlay，不写回 backend |
| `readActiveObjectGizmoFrame` / `syncActiveBackendRootFromObjectFrame(cur, dragging)` | 拖动时向 OSG 后代传播旋转 |
| `attachGizmoOverlayToActiveBackend` / `detachGizmoOverlay` | overlay 挂接 |
| `cacheSelectionGizmoPose` | 提交上次 gizmo 姿态 |
| `setBackendRootWorldMatrixFromWorld` | `local = world * inv(parentWorld)` |

### 5.4 拾取 — 对象

| 方法 | 说明 |
|------|------|
| `pickAndActivateBackendAtScreenPos` | 射线 → backendId |
| `bindBackendVisualRoot` / `unbindBackendVisualRoot` / `clearBackendVisualBindings` | 索引维护；BREP  overload 可传入 `brepArtifacts` |
| `resolveBackendIdFromPickedPath` | 拾取路径解析 |
| `setPickVisualAlias(logicalId, visualId)` | 装配子零件无独立 Geode 时，射线/索引 scope 映射到共享 visual |
| `resolvePickScopeBackendId(id)` | `queryPick` 前解析 alias；BREP 坐标变换亦用 visual id |

### 5.5 拾取 — 统一查询（Phase 4）

| 类型 / 方法 | 说明 |
|-------------|------|
| `PickKind` / `PickQuery` / `PickResult` / `PickPreviewState` | 见 `PickTypes.h` |
| `OsgScene::queryPick` | 点 / 面 / 边 / 对象统一入口；hover 与 click 同路径 |
| `OsgScene::kPointPickHitRadiusPx` 等 | 共用阈值（32px 点、18px 线边距、25px 点击容差） |
| Widget `ViewportGestureRecognizer` | click/drag/release 吞没与 clickHold |

### 5.6 拾取 — 点云索引（Phase 5）

| 方法 | 说明 |
|------|------|
| `PickSpatialIndex` | `bindBackendVisualRoot` 时从 Geode 构建 KD（**BREP 域跳过**） |
| `BackendPickIndexRegistry` | `backendId → { pointIndex, brepIndex, generation }`；BREP 仅建 `brepIndex`（`buildFromArtifacts`），不建 `meshIndex` |
| `BrepPickIndex` | 面/边射线拾取；`buildFromArtifacts` 复用 `BrepImportArtifacts`，bind 时不重 tessellate |
| `cachePickablePointsFromNode` | 优先从 registry 导入，避免选中时重扫 Geode |
| `pickPointAtScreenPos` / `pickNearestPointAtScreenPos` | 屏幕最近点（legacy，内部仍可用） |
| `pickPointByRayIntersection` | 射线拾取 |

### 5.7 拾取 — 网格

| 方法 | 说明 |
|------|------|
| `MeshTopologyIndex` | 绑定 Visual 时缓存三角 soup（局部坐标）；**BREP 域不使用** |
| `pickMeshFaceByRayIntersection` | 三角命中 + 共面合并（Mesh / 点云） |
| `pickMeshEdgeByRayIntersection` | 边段最近（`kMeshEdgeHitRadiusPx`） |
| `showMeshFaceHighlight` / `showMeshEdgeHighlight` / `hideMeshElementHighlight` | 高亮 overlay（mask `kMaskPickOverlay`） |

### 5.7a 拾取 — B-rep（`BrepModel`）

| 方法 / 字段 | 说明 |
|-------------|------|
| `OsgScene::tryQueryBrepPick` | 面/边/点 BREP 射线查询；`xformBackendId = resolvePickScopeBackendId(backendId)` |
| `stepModelPointToWorldMm` / `worldPointToStepModelMm` | 模型↔世界；读 `m_backendSkipCenterRebase`：装配 `skipInnerModelCenterRebase=true` 时**不再**加减 `modelCenter` |
| `m_backendSkipCenterRebase` | `upsertBackendBranchInScene` 写入；与 Visual 去心选项对齐，避免线/面高亮偏移 |
| `BackendPickDomain::Brep` | `BackendIdUserData` 标记；registry bind 时跳过 pointIndex |

装配导入：逻辑 part id 经 `setPickVisualAlias` 指向 `importParent` 的 visual id，保证 hover/click 命中共享 Geode 且高亮坐标正确。轨迹/AI 特征 overlay 经 `feature_pick_transform`（`IRobotOsgViewHost::resolvePickScopeBackendId`）走同一 visual id 与 skip-rebase 规则。

### 5.8 罗盘 Gizmo

| 方法 / 模块 | 说明 |
|------|------|
| `osg_compass::buildTransformCompassNode` | **对象选择与 TCP 示教共用**罗盘网格（实心 torus 环 + 正半轴）；`OsgCompassGeometry.h/.cpp` |
| `osg_compass::kCompassAxisLength` 等 | 与 `updateCompassScale` / `updateTcpTeachCompassScale` 共用缩放常量 |
| `createCompassNode` / `attachCompassGraphics` / `detachCompassGraphics` | 委托 `buildTransformCompassNode`；`m_compassScaleTransform` 仅缩放几何，避免 PAT scale 拉偏枢轴 |

**源文件**：`inc/OsgCompassGeometry.h`、`source/OsgCompassGeometry.cpp`（`OsgWidgetCore.vcxproj`）；几何基于 `osg/Shape` torus，勿依赖 `osg/Cone`。改罗盘后须先编 **OsgWidgetCore** 再链式编 Widget/RobotWidget。
| `syncCompassGizmoOrientation` | World：`compassAtt = R⁻¹`；Local：单位四元数 |
| `pickAxisAtScreenPos(mouseX, mouseY, preferRing, outPickedRing)` | 轴/环命中 → `kGizmoAxisX/Y/Z` |
| `computeCameraScreenRayWorld` | Qt 逻辑坐标 × DPR |
| `computeGizmoPivotWorld` / `logGizmoPivotDiagnostics` | inner 原点世界坐标；`POINTCLOUD_GIZMO_PIVOT_DIAG` |
| `gizmoCompassUnitAxisWorld` | 委托 `ObjectGizmoFrame::dragAxisDirectionSceneWorld`（**场景世界**，与 `computeGizmoPivotWorld` 同系） |
| `beginGizmoScreenDrag` / `gizmoScreenDragDs` | 平移：冻结屏幕轴 + `mmPerPixel`（与 TCP 示教同思路） |
| `beginGizmoScreenRotate` / `gizmoScreenRotateDeltaRad` | 旋转：绕冻结的 `m_gizmoRotatePivotWorld` 的屏幕角增量 |
| `gizmoScreenAngleAtMouse` | 在垂直于环法向的屏幕平面内 `atan2`；法向来自 `gizmoCompassUnitAxisWorld` |

**坐标系分工（对象 gizmo 旋转）**：

| 环节 | 坐标系 |
|------|--------|
| 屏幕转角、罗盘视觉法向 | 场景世界（`dragAxisDirectionSceneWorld`） |
| 写入 `outer` 四元数 | outer **父节点**局部（`dragAxisDirectionOuterParent`） |
| World 帧 | `R_new = Quat(δ, axisParent) * R_old`，`axisParent = worldDirectionToOuterParent(场景 X/Y/Z)` |
| Local 帧 | `R_new = R_old * Quat(δ, axisParent)`，`axisParent = R_old * localAxis` |

### 5.8 相机与注释

| 方法 | 说明 |
|------|------|
| `focusCameraOnBackend(backendId)` | 合并 `isBackendDescendantOf` 下各 OSG 分支世界包围球并移动 Trackball；空壳父（无几何）依赖逻辑父链；世界坐标顶点（`skipInnerModelCenterRebase`）用 `loc.center()` 变换求中心，勿仅用 outer 平移 |
| `hasPointAnnotations()` | 是否有注释（帧回调缩放） |

### 5.9 关键公共成员（控制器直接读写）

| 成员 | 用途 |
|------|------|
| `m_activeBackendId`, `m_activeBackendOuterPat` | 当前选中 |
| `m_backendParentIds`, `m_backendModelCenters`, `m_backendVisibility` | 每对象状态 |
| `m_backendSkipCenterRebase` | 与 `skipInnerModelCenterRebase` 对齐；BREP 拾取/高亮坐标变换 |
| `m_pickVisualAliases` | 逻辑 backendId → 实际承载 Geode 的 visual backendId |
| `m_backendVisualBindings` | `BackendVisualBindingIndex` |
| `m_backendPickIndexes` | `BackendPickIndexRegistry`（点云/网格拾取索引） |
| `m_selectionActive`, `m_objectSelectionMode`, `m_pointPickMode`, `m_mesh*PickMode` | 模式 |
| `m_dragging`, `m_rotating`, `m_dragAxis`, `m_hoverAxis` | 拖拽状态 |
| `m_annotations`, `m_pickablePointsLocal`, KD 树字段 | 注释与点拾取 |

### 5.10 嵌套类型

| 类型 | 说明 |
|------|------|
| `DragAxis` | `None`, `X`, `Y`, `Z` |
| `TransformGizmoFrame` | `World` / `Local` 罗盘对齐 |
| `AnnotationEntry` | 注释 OSG 节点 + `backendId` + 锚点 |
| `kGizmoAxisNone/X/Y/Z` | 拾取轴常量 |

---

## 6. 与 `Widget::OsgWidget` / Host 的分工

| 能力 | 所在层 |
|------|--------|
| Qt 事件、`eventFilter` | `OsgWidget`（**由 `CloudSimHost.dll` 编译**）+ `*Operation` / `*Controller` |
| 场景图、拾取、gizmo 数学 | `OsgScene`（本 DLL） |
| `IRobotBackendPoseSink` | `OsgWidget` 委托 `OsgScene` |
| 契约出口 `IRenderView` | Host `OsgRenderViewAdapter` 包装 `OsgWidget` |
| 矩阵/显隐/拾取（无 Qt） | `IBackendSceneBridge` → `OsgWidgetSceneBridge` → 本模块 API |

**UI 调用约定**（勿绕过）：

- 改后端 **pose/可见性**：`doc->data().applyPropertyChange` 或 gizmo 提交 → Host 写 Data 再 `syncOuterPatFromBackend`。
- 树选中加载分支：`BackendSceneDocumentFacade::ensureSelectionVisualForBackend`（内部 `load*FromBackendData` + `syncSelectionFromBackend`）。
- 插件/菜单 **新对象**：Host `DocumentImportFacade`，不在 UI 直接 `buildOuterBranch`。

---

## 7. 端到端：选中与拖拽

1. 拾取 → `resolveBackendIdFromPickedPath` → `syncGizmoAndPickFromBackend` → Host `publishSelectionChanged`
2. **LMB 平移** → `beginGizmoScreenDrag` → 每帧 `gizmoScreenDragDs` → `translateAlongWorldDirection(冻结轴)` → `applyToOuter`
3. **RMB 旋转** → `cacheRotatePivot`（世界枢轴）→ `beginGizmoScreenRotate` → `gizmoScreenRotateDeltaRad` → `adjustCenterPlusPoseForRotationDelta` → `applyToOuter`（拖动中**不** `emit selectedObjectPoseChanged`）
4. 释放 → `writeActiveBackendPoseFromOsg` → `publishPoseCommittedFromBackend` → `EventHub` → `MainWindow` 刷新属性面板

详见 [`../../ARCHITECTURE_SUMMARY.md`](../../ARCHITECTURE_SUMMARY.md) §6.2.0。

---

## 8. 相关文档

- 可视化构建：[`../BackendVisual/DEVELOPER_GUIDE.md`](../BackendVisual/DEVELOPER_GUIDE.md)
- Qt / Host 桥接：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md)（对象罗盘 §6.3.1–§6.3.2；TCP 示教 §13.1；属性/EventHub §12a）
- per-link FK / **M0·P**：[`../../Robot/RobotScene/DEVELOPER_GUIDE.md`](../../Robot/RobotScene/DEVELOPER_GUIDE.md) §8
- Host 组合根：[`../Host/CloudSimHost/DEVELOPER_GUIDE.md`](../Host/CloudSimHost/DEVELOPER_GUIDE.md)
