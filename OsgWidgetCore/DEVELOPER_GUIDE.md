# OsgWidgetCore 模块开发文档

## 1. 模块定位

`OsgWidgetCore` 是 **与 Qt 无关** 的 OSG 场景核心：分层场景根、相机导航、后端对象绑定与拾取、点/边/面拾取、注释、gizmo 罗盘、`ObjectGizmoFrame` 位姿数学。重绘通过 `setRequestRedraw` 回调由 `Widget::OsgWidget` 注入。

| 属性 | 说明 |
|------|------|
| 主类 | `OsgScene`（头文件 `OsgScene.h`，实现分 `OsgScene*.cpp`） |
| 依赖 | `BackendVisual`, `Data/BackendDataBase` |
| 导出 | `OSGWIDGETCORE_EXPORT` |

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

**语义**：outer 局部矩阵 = **`T(centerPlusPose) * R(attitude)`**（行向量 OSG）；文件原点在 inner 局部 (0,0,0)。

| 字段 / 方法 | 说明 |
|-------------|------|
| `modelCenter` | 与 `m_backendModelCenters` 一致 |
| `centerPlusPose` | `modelCenter + backend.pose` |
| `attitude` | outer 四元数 |
| `backendPoseRelativeToCenter()` | 即后端 `pose` |
| `fromOuter(outer, modelCenter, out)` | 从场景读帧 |
| `setFromBackend(poseRelCenter, attitude, modelCenter)` | 从后端写帧 |
| `applyToOuter(outer)` | 写回 `m_activeBackendOuterPat` |
| `translateAlongWorldAxis` / `translateAlongBodyAxis` | 平移 |
| `rotatePreMultiplyWorldAxis` / `rotatePostMultiplyLocalAxis` | 旋转 |
| `setRotationKeepingPivotInOuterParent` | 保枢轴旋转 |
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
| `bindBackendVisualRoot` / `unbindBackendVisualRoot` / `clearBackendVisualBindings` | 索引维护 |
| `resolveBackendIdFromPickedPath` | 拾取路径解析 |

### 5.5 拾取 — 点云

| 方法 | 说明 |
|------|------|
| `cachePickablePointsFromNode` | 提取顶点 |
| `pickPointAtScreenPos` / `pickNearestPointAtScreenPos` | 屏幕最近点 |
| `pickPointByRayIntersection` | 射线拾取 |
| `rebuildPointKdTree` / `nearestCandidatesByKdTree` | KD 加速 |

### 5.6 拾取 — 网格

| 方法 | 说明 |
|------|------|
| `pickMeshFaceByRayIntersection` | 三角命中 |
| `pickMeshEdgeByRayIntersection` | 边段最近 |
| `showMeshFaceHighlight` / `showMeshEdgeHighlight` / `hideMeshElementHighlight` | 高亮 overlay |

### 5.7 罗盘 Gizmo

| 方法 | 说明 |
|------|------|
| `createCompassNode` / `attachCompassGraphics` / `detachCompassGraphics` | 罗盘几何 |
| `pickAxisAtScreenPos(mouseX, mouseY, preferRing, outPickedRing)` | 轴/环命中 → `kGizmoAxisX/Y/Z` |
| `computeCameraScreenRayWorld` | Qt 逻辑坐标 × DPR |
| `computeGizmoPivotWorld` / `logGizmoPivotDiagnostics` | 枢轴；`POINTCLOUD_GIZMO_PIVOT_DIAG` |

### 5.8 相机与注释

| 方法 | 说明 |
|------|------|
| `focusCameraOnBackend(backendId)` | 对准外包络 |
| `hasPointAnnotations()` | 是否有注释（帧回调缩放） |

### 5.9 关键公共成员（控制器直接读写）

| 成员 | 用途 |
|------|------|
| `m_activeBackendId`, `m_activeBackendOuterPat` | 当前选中 |
| `m_backendParentIds`, `m_backendModelCenters`, `m_backendVisibility` | 每对象状态 |
| `m_backendVisualBindings` | `BackendVisualBindingIndex` |
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

## 6. 与 `Widget::OsgWidget` 的分工

| 能力 | 所在层 |
|------|--------|
| Qt 事件、`eventFilter` | `OsgWidget` + `*Operation` / `*Controller` |
| 场景图、拾取、gizmo 数学 | `OsgScene` |
| `IRobotBackendPoseSink` | `OsgWidget` 委托 `OsgScene` |

---

## 7. 端到端：选中与拖拽

1. 拾取 → `resolveBackendIdFromPickedPath` → `syncGizmoAndPickFromBackend`
2. 拖拽 → `ObjectTransformOperation` 改 `ObjectGizmoFrame` → `applyToOuter` → `syncActiveBackendRootFromObjectFrame(..., true)`
3. 释放 → `cacheSelectionGizmoPose` → 信号 `transformGizmoCommitted`

详见 [`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §6.2.0。

---

## 8. 相关文档

- 可视化构建：[`../BackendVisual/DEVELOPER_GUIDE.md`](../BackendVisual/DEVELOPER_GUIDE.md)
- Qt 桥接：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md)
