# OsgWidgetCore 模块开发文档

> **空间契约**：[`../../../docs/spatial_contract_world_pose.md`](../../../docs/spatial_contract_world_pose.md) §1.1 — gizmo 读写总位姿；内旋 ZYX 存盘；Gizmo World/Local 仅交互方式不同；`modelCenter` 仅聚焦与外包络。

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

**语义（契约 §1.1）**：`centerPlusPose` = **`backend.pose`**（模型原点在外层父系）；`attitude` = 内旋 ZYX 总旋转；outer 矩阵 = `osgMatrixFromRigidTransform`（行向量，主动 `p'=p×R+pose`）。绕模型原点旋转时 **只改 `attitude`，不改 `centerPlusPose`**。

**`setFromBackend`**：`centerPlusPose = backend.pose`（**不**加 `modelCenter`）。**`fromOuter`**：`rigidTransformFromOsg(outer->getMatrix())` 反解 pose + 四元数。

| 字段 / 方法 | 说明 |
|-------------|------|
| `centerPlusPose` | 即 **`backend.pose`**（模型原点，非 bbox 中心） |
| `attitude` | outer 旋转四元数（内旋 ZYX 总姿态） |
| `modelCenter` | 外包络缓存；gizmo **不**参与读写 |
| `backendPoseRelativeToCenter()` | 与 `backend.pose` 同义（inner=0） |
| `fromOuter` / `applyToOuter` | `rigidTransformFromOsg` / `osgMatrixFromRigidTransform` |
| `translateAlongWorldDirection` | 沿世界方向改 `centerPlusPose`（改 pose） |
| `translateAlongWorldAxis` / `translateAlongBodyAxis` | 沿世界/物体轴平移 pose |
| `dragAxisDirectionSceneWorld` / `dragAxisDirectionOuterParent` | World/Local 罗盘轴方向（交互，见 §1.1 Gizmo 表） |
| `adjustCenterPlusPoseForRotationDelta` | 绕模型原点：仅 `m_attitude = R_new` |
| `setRotationKeepingPivotInOuterParent` | 同上，仅更新 attitude |
| `rotatePreMultiplyWorldAxis` | 绕**世界轴**增量（外旋式交互） |
| `rotatePostMultiplyLocalAxis` | 绕**物体轴**增量（内旋式交互） |
| `pivotWorldFromOuter` / `pivotInOuterParentFromOuter` | 模型原点世界/父系坐标（诊断） |

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
| `setViewportPixels(w,h)` / `setDevicePixelRatio(dpr)` | 逻辑视口与 HiDPI 比例（`dpr = fbW / logicalW`） |

**屏幕坐标约定（HiDPI 必读）**

| 用途 | 坐标空间 | 入口 |
|------|----------|------|
| 罗盘 gizmo / TCP 示教：`projectToScreen`、`pickAxisAtScreenPos`、`gizmoScreenDragDs`、`computeCameraScreenRayWorld` | **逻辑像素**（与 `viewportWidth/Height`、Qt `QMouseEvent::pos()` 一致） | 传入 Qt 逻辑坐标，**勿再乘 DPR** |
| 点云/网格屏幕距离、`kPointPickHitRadiusPx` 等阈值 | **逻辑像素** | 与 `projectToScreen` 输出同系 |
| OSG `LineSegmentIntersector::WINDOW` 射线拾取 | **设备像素**（对齐主相机 `setViewport(0,0,fbW,fbH)`） | `logicalMouseToPickWindowCoords` |
| osgGA 轨道相机（`QWidgetViewer` 转发） | **设备像素** | `deviceCoord(logical, _devicePixelRatio)` |

主相机渲染 viewport 为设备像素；`OsgWidget::syncViewportLayoutFromFramebuffer` 同时写入逻辑 `setViewportPixels` 与 OSG viewport。修改 gizmo/拾取时勿混用「逻辑投影 + 设备鼠标」。

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
| `OsgScene::kPointPickHitRadiusPx` 等 | 共用阈值（32px 点、18px 线边距、25px 点击容差），均为 **逻辑像素** |
| Widget `ViewportGestureRecognizer` | click/drag/release 吞没与 clickHold |

### 5.6 拾取 — 点云索引（Phase 5）

| 方法 | 说明 |
|------|------|
| `PickSpatialIndex` | `bindBackendVisualRoot` 时从 Geode 构建 KD（**BREP 域跳过**） |
| `BackendPickIndexRegistry` | `backendId → { pointIndex, brepIndex, generation }`；BREP 仅建 `brepIndex`；bind 前 `ensureBrepImportPickArtifacts`（Phase2） |
| `BrepPickIndex` | 面/边射线拾取；`buildFromArtifacts` 复用 `BrepImportArtifacts`；无 artifacts 时 `build()` 仍会全量 tessellate（应避免） |
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
| `stepModelPointToWorldMm` / `worldPointToStepModelMm` | 模型↔世界：经 `getBackendRootWorldMatrix`（**不**加减 `modelCenter`） |
| `m_backendSkipCenterRebase` | **遗留**旁路表；`backendSkipsInnerModelCenterRebase` 恒 `false` |
| `BackendPickDomain::Brep` | `BackendIdUserData` 标记；registry bind 时跳过 pointIndex |

装配导入：逻辑 part id 经 `setPickVisualAlias` 指向 `importParent` 的 visual id，保证 hover/click 命中共享 Geode 且高亮坐标正确。轨迹/AI 特征 overlay 经 `feature_pick_transform`（`IRobotOsgViewHost::resolvePickScopeBackendId`）走同一 visual id 与 skip-rebase 规则。

### 5.8 罗盘 Gizmo

| 方法 / 模块 | 说明 |
|------|------|
| `osg_compass::buildTransformCompassNode` | **对象选择与 TCP 示教共用**罗盘网格（实心 torus 环 + 正半轴）；`OsgCompassGeometry.h/.cpp` |
| `osg_compass::kCompassAxisLength` 等 | 与 `updateCompassScale` / `updateTcpTeachCompassScale` 共用缩放常量 |
| `createCompassNode` / `attachCompassGraphics` / `detachCompassGraphics` | 委托 `buildTransformCompassNode`；`m_compassScaleTransform` 仅缩放几何，避免 PAT scale 拉偏枢轴 |
| `syncCompassGizmoOrientation` | World：`compassAtt = R⁻¹`；Local：单位四元数 |
| `pickAxisAtScreenPos(mouseX, mouseY, preferRing, outPickedRing)` | 轴/环命中 → `kGizmoAxisX/Y/Z`（逻辑像素） |
| `computeCameraScreenRayWorld` | Qt **逻辑**鼠标 → 世界射线（NDC 用 `mouse / viewportWidth`） |
| `computeGizmoPivotWorld` / `logGizmoPivotDiagnostics` | inner 原点世界坐标；`POINTCLOUD_GIZMO_PIVOT_DIAG` |
| `gizmoCompassUnitAxisWorld` | 委托 `ObjectGizmoFrame::dragAxisDirectionSceneWorld`（**场景世界**，与 `computeGizmoPivotWorld` 同系） |
| `beginGizmoScreenDrag` / `gizmoScreenDragDs` | 平移：冻结屏幕轴 + `mmPerPixel`（与 TCP 示教同思路） |
| `beginGizmoScreenRotate` / `gizmoScreenRotateDeltaRad` | 旋转：绕冻结的 `m_gizmoRotatePivotWorld` 的屏幕角增量 |
| `gizmoScreenAngleAtMouse` | 在垂直于环法向的屏幕平面内 `atan2`；法向来自 `gizmoCompassUnitAxisWorld` |

**源文件**：`inc/OsgCompassGeometry.h`、`source/OsgCompassGeometry.cpp`（`OsgWidgetCore.vcxproj`）；几何基于 `osg/Shape` torus，勿依赖 `osg/Cone`。改罗盘后须先编 **OsgWidgetCore** 再链式编 Widget/RobotWidget。

### 5.8b Mesh 轨迹 overlay（截面 / 拟合面 / raw 折线）

几何：`OsgSectionPlaneGeometry`（`buildSectionPlaneQuadNode`）；Widget 层 `OsgWidgetMeshSectionPlane.cpp`；拟合面 geode 在 `OsgScene::initSceneGraph`。

| 节点 / API | 说明 |
|------------|------|
| `showMeshSectionPlane` | 半透明截面片 + 可选罗盘；平面片挂 `backendObjectsGroup`，`Depth::LEQUAL` 参与遮挡 |
| `beginMeshSectionPlaneEdit` | 罗盘 `applyUnlitHighlitStateSet`（不参与深度遮挡，始终可见） |
| `showMeshFittedSurfacePreview` | `m_meshFittedSurfaceOverlayGroup` 上三角 soup，绿色半透明 |
| `setRawTrajectoryOverlay(..., segmentEndExclusive)` | 每段独立 `LINE_STRIP` + 全点 POINTS；空 segment 列表时单条折线 |
| `setInstructionPoseAxes` / `setRawTrajectoryOverlayFrames` | 实现于 `Widget/source/OsgWidget.cpp`：可见路点**合并为 1 个 Geode**（`POINTS` 原点 + `LINES` XYZ），勿再为每点建 `MatrixTransform`+`ShapeDrawable` 球；增删指令靠编排层全量 `set*` 重建 |
| `setReachableWorkspaceOverlay` / `clearReachableWorkspaceOverlay` | 半透明体素方块（可达域）；独立 Geode，勿与轨迹 overlay 混用 |

UI 同步见 RobotWidget「路点轴 OSG 绘制」与 §Mesh 轨迹生成（`syncSectionPlanePreview` / `syncBsplineSurfacePreview`）。

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
| `focusCameraOnBackend(backendId)` | 合并逻辑子树下各分支世界包围球；世界坐标顶点用 Geode 变换求中心，勿仅用 outer 平移 |
| `hasPointAnnotations()` | 是否有注释（帧回调缩放） |

### 5.9 关键公共成员（控制器直接读写）

| 成员 | 用途 |
|------|------|
| `m_activeBackendId`, `m_activeBackendOuterPat` | 当前选中 |
| `m_backendParentIds`, `m_backendModelCenters`, `m_backendVisibility` | 每对象状态 |
| `m_backendSkipCenterRebase` | 遗留；拾取/高亮经 `getBackendRootWorldMatrix` |
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
3. **RMB 旋转** → `cacheRotatePivot`（世界枢轴）→ `beginGizmoScreenRotate` → `gizmoScreenRotateDeltaRad` → `adjustCenterPlusPoseForRotationDelta` → `applyToOuter` → `selectedObjectRotationChanged`
4. 拖动中 → `MainWindow::syncPropertyPanelGizmoLiveValues` 从 gizmo 直写属性行；释放 → `writeActiveBackendPoseFromOsg` → `transformGizmoCommitted` → 全量 `updatePropertyPanel`

详见 [文档索引](../../../docs/README.md)「近期热点」与 Widget §13.1（TCP 示教）。

---

## 变更历史（2026-06）

### 屏幕坐标 / HiDPI
- gizmo、TCP 示教、屏幕投影拾取统一为 **逻辑像素**；DPR 修正后不再对 Qt 鼠标二次乘 `devicePixelRatio`。
- OSG `WINDOW` 拾取与 `QWidgetViewer` 仍用设备像素（见 §5.1 屏幕坐标约定）。

---

## 8. 相关文档

- 可视化构建：[`../BackendVisual/DEVELOPER_GUIDE.md`](../BackendVisual/DEVELOPER_GUIDE.md)
- Qt / Host 桥接：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md)（TCP 示教 §13.1；Units/主题 §4.6、§5.3）
- per-link FK / **M0·P**：[`../../Robot/RobotScene/DEVELOPER_GUIDE.md`](../../Robot/RobotScene/DEVELOPER_GUIDE.md) §8
- Host 组合根：[`../Host/CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md)
