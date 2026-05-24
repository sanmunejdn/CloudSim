# Data 模块开发文档

## 1. 模块定位

`Data` 是应用内 **后端数据真源（Single Source of Truth）**：统一描述场景中的点云/网格对象、属性面板协议、DAG 父子关系、可选跟随约束组件。不处理 Qt 事件、不持有 OSG 节点。

| 属性 | 说明 |
|------|------|
| x64 输出 | `Data.dll`（`DATA_LIB`） |
| 静态嵌入 | `PointCloudAlgorithm.lib`（CGAL 算法，无独立 DLL） |
| 动态链接 | x64：`RunLogger.dll`（import lib） |
| 并发 | `BackendDataManager` 容器与边由 `std::shared_mutex` 保护；**单个 `BackendDataBase` 字段**无细粒度锁，跨线程写需调用方序列化（通常 UI 线程） |
| 序列化 | 属性行 JSON + 工程 **`project.json` v4**（`saveToJson`/`loadFromJson`、内嵌 `geometry`、`propertyBag`、`components`） |
| 导出 | `DATA_EXPORT` |

---

## 2. 基础值类型（`BackendDataBase.h` / `BackendFollowMath.h`）

| 类型 | 字段 / 说明 |
|------|-------------|
| `BackendVec3` | `x,y,z` double |
| `BackendBoundingBox` | `min`, `max`, `valid` |
| `BackendColor` | `r,g,b,a` float 0..1 |
| `BackendPoseValue` | `position` + `eulerDeg` |
| `BackendPoseReferenceFrame` | `World` / `Parent` |
| `BackendMat4` | `v[16]` 列主序（遗留）；`backend_mat4_multiply` 用 Eigen 列向量语义（与 GeometryEngine `composeColumn` 一致） |
| `backend_world_mat_from_pose` 等 | 与 Visual 外层矩阵 `T(center+pose)*R` 一致；新刚体链在模块边界用 GeometryEngine（工具法兰×工具见 [`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md)，勿与 `composeScene` 混用） |

---

## 3. `class BackendDataBase`（抽象根）

### 3.1 身份与类型

| 方法 | 说明 |
|------|------|
| `id()` / `setId` | 稳定 UUID 风格 id |
| `name()` / `setName` | 显示名（跟随 `follow.targetName` 匹配用） |
| `className()` | **纯虚**；如 `"PointCloudBackendData"`, `"Model"` |

### 3.2 几何契约（纯虚）

| 方法 | 说明 |
|------|------|
| `hasGeometry()` | 是否可渲染 |
| `geometryBounds()` | 模型空间 AABB |
| `geometryElementCount()` | 点数或三角数 |
| `clearGeometry()` | 清空缓冲 |

### 3.3 位姿与颜色（可覆盖）

| 方法 | 默认 |
|------|------|
| `pose` / `setPose`, `rotation` / `setRotation`, `color` / `setColor` | 零/白 |
| `hasPoseProperty()` 等 | `false` |
| `supportsBackendTransform()` | `hasPoseProperty()` |
| `applyBackendWorldPose(centerWorld, eulerDegWorld)` | 世界系写回窄接口 |

### 3.4 参考系与 4×4 矩阵

| 方法 | 说明 |
|------|------|
| `poseReferenceFrame()` / `setPoseReferenceFrame` | World / Parent |
| `poseInFrame` / `setPoseInFrame`（含 rotation） | 需 `BackendDataManager` 做父链变换 |
| `poseValue` / `setPoseValue` | `BackendPoseValue` 原子读写 |
| `worldMatrix(mgr)` / `setWorldMatrix(world, mgr)` | 缓存世界矩阵（实现内带锁） |
| `validatePoseFrameRoundTrip(mgr, epsilon)` | 帧转换自检 |

### 3.5 属性系统

| 方法 | 说明 |
|------|------|
| `propertyBag()` | `PropertyBag` 键值（类型安全 variant） |
| `snapshotPropertyRows(mgr)` | 属性面板 JSON 行数组 |
| `applyPropertyChange(key, value, errMsg, mgr)` | 单行提交（**UI/插件**应优先 `IDataService::applyPropertyChange`，由 Host `BackendVisualSync` 同步 OSG 并发布 `PoseCommitted`） |

### 3.6 组件（Component）

| 方法 | 说明 |
|------|------|
| `addComponent` / `removeComponent` / `getComponent` / `hasComponent` | 按 `componentType()` 字符串 |
| `getComponent<T>()` / `emplaceComponent<T>(...)` | 类型安全 |
| `listComponents()` | 全部组件 |

### 3.7 层级（经 Manager 图）

| 方法 | 说明 |
|------|------|
| `parentObjects(manager)` | 直接父对象 shared_ptr 列表 |
| `childObjects(manager)` | 直接子对象 |
| `descendantObjects(manager)` | 可达后代（DAG） |

### 3.8 工程持久化（`project.json` v4）

| 方法 | 说明 |
|------|------|
| `saveToJson() const` | 模板方法：写出公共字段 + `propertyBag` + `components` + 调用 `saveDerivedJson` |
| `loadFromJson(in, errMsg)` | 模板方法：恢复公共字段 + `propertyBag` + `components` + 调用 `loadDerivedJson`；兼容旧字段 `followAttachment` |
| `saveDerivedJson(out)` / `loadDerivedJson(in, errMsg)` | 派生类扩展（几何等），默认空实现 |

**公共字段**（基类统一）：`id`、`name`、`className`、`pose`、`rotation`、`color`、`worldMatrix`、`poseReferenceFrame`、`propertyBag`。

**派生扩展**：

| 类型 | `geometry` 字段 |
|------|-----------------|
| `PointCloudBackendData` | `kind=points`，`xyzBase64`，可选 `rgbaPerVertexBase64` |
| `MeshBackendData` | `kind=triangles`，`xyzBase64`；另 `mesh.transformPivotAtOrigin` |

仍保留 `writeProjectEmbeddedGeometry` / `readProjectEmbeddedGeometry` 供派生类内部使用。

---

## 4. 具体后端类型

### 4.1 `PointCloudBackendData`

| 注册名 | `className()` = `"PointCloudBackendData"` |
|--------|-------------------------------------------|

| 方法 | 说明 |
|------|------|
| `setPointBuffers(xyz, rgbaPerVertex)` | `3*N` float + 可选 `4*N` RGBA |
| `pointPositionsXyz()` / `pointVertexRgba()` | 只读缓冲 |
| `loadFromFile` | `.ply`, `.xyz`（CGAL） |
| `readPointCloudFromPlyFile` / `writePointCloudPlySidecar` | PLY 专用 |
| `writeProjectEmbeddedGeometry` / `readProjectEmbeddedGeometry` | 工程内嵌 Base64 |

位姿/颜色/属性：`hasPoseProperty` 等均为 `true`。

### 4.2 `MeshBackendData`

| 注册名 | `className()` = **`"Model"`**（显示名 Mesh） |
|--------|-----------------------------------------------|

| 方法 | 说明 |
|------|------|
| `setTriangleSoup` / `triangleSoup()` | 每三角 9 float（v0,v1,v2 各 xyz） |
| `setTriangleSoupWithNormals` / `triangleVertexNormals()` / `hasTriangleVertexNormals()` | 可选每顶点法线（9 float/三角，与 soup 下标对齐）；OBJ 含 `vn` 时写入 |
| `transformVerticesColumnMajorHomogeneous4x4(colMajor16)` | URDF：mesh 文件系 → 连杆系；**同时**变换 `triangleVertexNormals`（3×3 旋转，无平移） |
| `setTransformPivotAtOrigin(true)` | 枢轴在原点；`modelCenter` 为 (0,0,0) |
| `loadFromFile` | 见 §4.2.1 |
| `loadStepHierarchyFromFile` / `loadDxfHierarchyFromFile` | 静态，输出 `MeshHierarchyPart` 列表 |

### 4.2.1 网格文件导入（`loadFromFile`）

统一写入 `triangleSoup`；`clearGeometry()` 同时清空 soup 与法线缓冲。

| 扩展名 | 实现 | 绕序 / 法线 |
|--------|------|-------------|
| `.step` / `.stp` | OCCT `STEPControl_Reader` + `BRepMesh_IncrementalMesh` | `TopAbs_REVERSED` 面导出时交换三角 n2/n3，避免场景光照发黑 |
| `.obj`（含 `vn`） | 自研解析 `v` / `vn` / `f v/vt/vn` | **保留文件顶点顺序**；`vn` 写入 `triangleVertexNormals`，供 `MeshBackendVisual` 光照（CGAL `read_polygon_soup` 会丢弃 `vn`） |
| `.obj`（无 `vn`） | CGAL `read_polygon_soup` | `orient_polygon_soup` 后按顶点质心外向翻转扇形三角绕序 |
| `.stl` / `.ply` / `.off` | CGAL `read_polygon_soup` | 同 `.obj` 无 `vn` 路径 |

**现象与约定**：`BackendVisual` 在 `useSceneLighting=true` 时，无法线缓冲则用法线 `n = (p1-p0)×(p2-p0)`。绕序错误或仅用绕序对齐内向 `vn` 会导致**整面发黑**；带 `vn` 的 CAD/Max OBJ 必须走文件法线路径。

**Widget 导入**：`MainWindowImportCaptureRenderController` 对 STEP 优先 `loadStepHierarchyFromFile`（多零件层级）；单件 STEP 与上述 `loadFromFile` 一致。`.obj` 等不走 OSG `importModelFile` fallback（见该控制器注释）。

### 4.3 `struct MeshHierarchyPart`

STEP/DXF 层级导入中间结构：`partPath`, `parentPartPath`, `displayName`, `triangleSoup`。

DXF 分件经 `dxfExpandInsertRecursive` 写入的 `triangleSoup` 通常为 **世界坐标**；Host 导入时用 `skipInnerModelCenterRebase` 且**不做** Follow 求解（见 [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) §4.4.1a）。

---

## 5. 属性基础设施

### 5.1 `PropertyBag`

| API | 说明 |
|-----|------|
| `set<T>(name, value)` | 类型化键 `PropertyKey{name, type_index}` |
| `tryGet<T>(name, out)` | 类型不匹配返回 false |
| `applyDiff(PropertyBagDiff)` | 批量更新 |
| `toJson()` | 序列化 |

### 5.2 `backend_property_json`（`BackendPropertyRow.h`）

行格式：`{ key, labelEn, editable, value }`。

### 5.3 `backend_property_schema`（`BackendPropertySchema.h`）

| 函数 | 产出 |
|------|------|
| `pointCloudBackendSchema()` / `meshBackendSchema()` | PropertyCore `PropertySchema` |
| `followAttachmentBackendPropertySchema()` | `follow.targetName` 等 |
| `schemaForBackendClassName(className)` | 分发 |
| `tagPoseRotationColorSemantics` | `pose.*` → 影响世界变换；`color.*` → 仅颜色 |

### 5.4 `BackendAttributeBase` 工厂

| 工厂 | 绑定属性 |
|------|----------|
| `makeBackendPoseAttribute()` | pose.x/y/z |
| `makeBackendRotationAttribute()` | rotation.x/y/z |
| `makeBackendDisplayColorAttribute()` | color.r/g/b/a |

---

## 6. `FollowAttachmentComponent`（`IBackendComponent`）

**类型键**：`"FollowAttachment"`

| 方法 | 说明 |
|------|------|
| `enabled` / `targetBackendId` / `localPosition` / `localEulerDeg` | 跟随约束参数 |
| `solverPaused` | gizmo 拖动时暂停求解写回 |
| `hierarchyDriven` | 由 `attachChild` 自动建立的目标 |
| `appendPropertyRows` / `applyPropertyChange` | UI：`follow.targetName` → `findByName` |
| `writeJson` / `readJson` | 组件数据体（经 `BackendComponentCodecRegistry` 写入 `components[]`） |
| `recomputeLocalFromCurrentWorld`（静态） | 从当前世界位姿重算局部偏移 |

---

## 6.1 `BackendComponentCodecRegistry` + `BackendComponentCodecBuiltins`

| API | 说明 |
|-----|------|
| `registerCodec(type, writer, reader)` | 注册组件编解码 |
| `encodeComponent(component)` | → `{ type, data }` |
| `decodeComponent(entry)` | 按 `type` 还原 `BackendComponentPtr` |
| `setWarningHook` | 重复注册、未知类型、编解码失败 → 默认接 `RunLogger::warn` |
| `ensureBackendComponentCodecBuiltinsRegistered()` | 内建注册 `FollowAttachment`（`BackendComponentCodecBuiltins.h`） |

新增组件：实现 `writeJson`/`readJson`，在 builtins 中 `registerCodec`，无需改 `MainWindowProjectIo`。

---

## 7. `BackendFollowTransformSolver`

| API | 说明 |
|-----|------|
| `WorldMatQuery` | `bool(backendId, BackendMat4& outWorld)`，优先 OSG 真值 |
| `solve(mgr, worldQuery, skipUpdatingFollowerId, limitPoseUpdateToFollowerIds)` | 拓扑序更新 follower 的 `pose/rotation` |

与 `Widget::runBackendFollowSolveAndSync`、`BackendSceneDocumentFacade` 脏集配合。

---

## 8. `BackendDataManager`（场景注册表 + DAG）

### 8.1 对象注册

| 方法 | 说明 |
|------|------|
| `instance()` | 单例（每文档可独立实例，由 `DocumentPage` 持有） |
| `registerData` / `unregisterData` | 注册/移除（写锁） |
| `getData` / `contains` / `listData` | 查询（读锁） |
| `findByName` / `findByClass` / `findByComponent` | 条件查找 |

### 8.2 边与图算法

| 方法 | 说明 |
|------|------|
| `attachChild(parentId, childId)` | 有向边 |
| `setParent(childId, parentId)` | 单父替换 |
| `detachChild` / `detachAllParents` | 删边 |
| `parentsOf` / `childrenOf` / `ancestorsOf` / `descendantsOf` | id 列表 |
| `subtreeIds(rootId)` | 缓存：根 + 后代 |
| `topoOrder()` / `rootIds()` / `listEdges()` | 拓扑与根 |
| `wouldCreateCycle` / `validateGraph` | 一致性 |

### 8.3 观测与调试

| 方法 | 说明 |
|------|------|
| `addHierarchyObserver` / `removeHierarchyObserver` | **禁止**在回调内再抢写锁 |
| `takeSnapshot()` | `BackendSnapshot` 向量 |
| `collectBaselineMetrics` | 性能基线 |
| `clear()` | 清空 |

### 8.4 `BackendHierarchyModel`

UI 侧增量镜像：`resyncFrom(mgr)`，`subtreeIds(root)`（结构变更后缓存失效）。

---

## 9. `BackendRegistry`（类型工厂，非场景）

| 方法 | 说明 |
|------|------|
| `registerType(BackendMeta)` | `className`, `displayName`, `factory`, 标志 |
| `create(className)` | `shared_ptr<BackendDataBase>` |
| `ensureBackendBuiltinsRegistered()` | `PointCloudBackendData` + `Model`（`MeshBackendData`） |

工程加载时：`MainWindowProjectIo` 按 JSON 中 `className` 调用 `create`，再 `loadFromJson`。

---

## 10. 工具

| 模块 | 作用 |
|------|------|
| `geometry_base64` | 浮点缓冲 ↔ Base64（工程嵌入） |
| `backend_relations` | `parents`/`children` 便捷包装 |
| `property_rows_compat::syncTransformColorToBag` | 面板 ↔ PropertyBag 同步 |

---

## 11. 与上层模块边界

| 上层 | 如何使用 Data |
|------|----------------|
| `Widget` / `MainWindow` | 属性：`doc->data().applyPropertyChange`；注册/导入：Host `DocumentImportFacade`；场景：`BackendSceneDocumentFacade` |
| `CloudSimPluginHost` | 插件经 SDK；宿主内 `unregisterSubtree`、`importFileIntoActiveDocument`、`registerAdoptedMesh`（见 [`../CloudSimPluginHost/DEVELOPER_GUIDE.md`](../CloudSimPluginHost/DEVELOPER_GUIDE.md)） |
| `BackendVisual` | 读几何缓冲建 OSG |
| `PointCloudAlgorithm` | 经 `PointCloudBackendOps` 做点云下采样/变换/重建网格（见 [`../PointCloudAlgorithm/DEVELOPER_GUIDE.md`](../PointCloudAlgorithm/DEVELOPER_GUIDE.md)） |
| `RobotUrdf` | 每连杆 `MeshBackendData` |
| `RobotScene` | 读关节/连杆 id，写 `worldMatrix` |

**原则**：`Data` 为真源；跨模块勿长期 `BackendDataBase::applyPropertyChange` + 手写 OSG sync，应走 [`CloudSimCore` `IDataService`](../../Contracts/CloudSimCore/DEVELOPER_GUIDE.md) 或 Host Facade。

**Data 不包含 OSG 头文件**；世界矩阵经 `IBackendSceneBridge` 列主序 16 double 与 OSG 对齐。

---

## 12. 相关文档

- 可视化：[`../BackendVisual/DEVELOPER_GUIDE.md`](../BackendVisual/DEVELOPER_GUIDE.md)（法线光照 §4.2）
- 场景门面 / 文件导入 / 工程 I/O：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) §6.1、§11；插件宿主：[`../CloudSimPluginHost/DEVELOPER_GUIDE.md`](../CloudSimPluginHost/DEVELOPER_GUIDE.md)
- 总架构：[`../../ARCHITECTURE_SUMMARY.md`](../../ARCHITECTURE_SUMMARY.md) §4.3、§6.5
- 持久化设计/任务/回归：[`../../docs/backend_persistence/`](../../docs/backend_persistence/)
