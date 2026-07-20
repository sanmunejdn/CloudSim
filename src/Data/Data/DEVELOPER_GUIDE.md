# Data 模块开发文档

> **空间契约 v2**：[`../../../docs/spatial_contract_world_pose.md`](../../../docs/spatial_contract_world_pose.md) — **Breaking**：JSON 仅 `worldMatrix`（16 元）；`pose`/`rotation` 为分解视图；`p_world = p_geometry × worldMatrix`。

### Breaking: v2 坐标模型

| 变更 | 说明 |
|------|------|
| 权威存储 | `BackendDataBase::m_worldMatrix`；`setPose`/`setRotation` 重建矩阵 |
| JSON | `saveToJson` 只写 `worldMatrix`；缺字段 `loadFromJson` 失败并提示重导入 |
| 配准 | 世界系 ICP → `icpDeltaWorld`；不输出 `alignedTemplateShape` |
| 面重构 | `scanPointsToTemplateModelFrame`：`v_model = v_geo × M_scan × inv(M_tpl)` |
| UI 工具 | `composeWorldMatrix` / `decomposeWorldMatrix`（`BackendFollowMath.h` 别名） |

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
| `objectWorldMatrix` / `transformPointToWorld` | 世界点 = `objectWorldMatrix × v_stored`（见 `BackendSpatial.h`） |
| `backend_world_mat_from_pose` | 列向量 `p'=R×p_model+pose`；委托 `rigidTransformFromBackendPoseEuler` + `colMajorFromRigidTransform`；**无** `+modelCenter`（见契约 §1.1） |

---

## 3. `class BackendDataBase`（抽象根）

### 3.1 身份与类型

| 方法 | 说明 |
|------|------|
| `id()` / `setId` | 稳定 UUID 风格 id |
| `name()` / `setName` | 显示名（跟随 `follow.targetName` 匹配用） |
| `isVisible()` / `setVisible` | 场景显示/隐藏真源（持久化字段 `visible`） |
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

**公共字段**（基类统一）：`id`、`name`、`className`、`visible`、`pose`、`rotation`、`color`、`worldMatrix`、`poseReferenceFrame`、`propertyBag`。

| 字段 | 说明 |
|------|------|
| `visible` | 场景显示态真源（默认 `true`）；缺字段加载时视为显示。OSG NodeMask / 后端树勾选为派生视图 |

**派生扩展**：

| 类型 | `geometry` 字段 |
|------|-----------------|
| `PointCloudBackendData` | `kind=points`，`storage=ply_sidecar`，`pointCount`；几何真源 `objects/{id}.ply`（兼容旧工程 `xyzBase64`） |
| `MeshBackendData` | `kind=triangles`，`xyzBase64`；另 `mesh.transformPivotAtOrigin` |

仍保留 `writeProjectEmbeddedGeometry` / `readProjectEmbeddedGeometry` 供派生类内部使用。

---

## 4. 具体后端类型

### 4.0 文件路径编码约定

Data 层凡以 `std::string path` 打开磁盘文件的 API（含 `PlyIo`、`PointCloudBackendData::loadFromFile` / `readPointCloudFromPlyFile`、`MeshBackendData::loadFromFile` 传入 CGAL/OCCT 的路径）均约定为 **Qt 本地窄字节路径**，与 `QFile::encodeName(QString)` 一致。

| 做法 | 说明 |
|------|------|
| Widget/Host 传参 | `QByteArray enc = QFile::encodeName(filePath)` → `std::string(enc.constData(), enc.size())` |
| C++ 打开文件 | `std::ifstream{std::filesystem::path{path}}` 或 `std::filesystem::path(path)` |
| **禁止** | 对本地路径使用 `std::filesystem::u8path`（中文 Windows 下非法 UTF-8 序列会抛 `system_error`） |
| **禁止** | Widget 侧对含中文路径用 `filePath.toUtf8()` 再交给上述 API（与 `encodeName` 语义不一致） |

### 4.1 `PointCloudBackendData`

| 注册名 | `className()` = `"PointCloudBackendData"` |
|--------|-------------------------------------------|

| 方法 | 说明 |
|------|------|
| `setPointBuffers(xyz, rgba)` / `setPointBuffers(xyz, rgba, normals)` | `3*N` float + 可选 `4*N` RGBA + 可选 `3*N` 法线 |
| `pointNormalsNxNyNz()` / `hasPointNormals()` | 法线缓冲（**v1 不写入 project.json**，仅内存） |
| `pointPositionsXyz()` / `pointVertexRgba()` | 只读缓冲 |
| `loadFromFile` | `.ply`, `.xyz`（CGAL）；路径见 §4.0 |
| `readPointCloudFromPlyFile` / `writePointCloudPlySidecar` | PLY 专用（**仅顶点**，忽略 `element face`）；路径见 §4.0 |
| `writeProjectEmbeddedGeometry` / `readProjectEmbeddedGeometry` | 旧工程内嵌 Base64（新保存走 PLY sidecar） |
| `writePointCloudPlySidecar` / `readPointCloudPlySidecar` | 工程 `objects/{id}.ply` 读写 |

**PLY 双形态（`PlyIo.h`，路径 §4.0）**

| API | 说明 |
|-----|------|
| `scanPlyHeader` | 解析头：`vertexCount`、`faceCount`、`hasFaceElement`、点云读路径所需 x/y/z/RGB 列索引；首行可剥离 UTF-8 BOM |
| `plyFileHasTriangleFaces` | `valid && hasFaceElement && faceCount > 0`；面元元素名支持 `face` / `polygon` / `triangle` |
| 分流约定 | 含三角面的 PLY 经点云菜单或 Host `importPointCloudFile` 时，由 Host/Widget 改道 `MeshBackendData::loadFromFile`（`read_polygon_soup`），**不在此**读顶点 |
| `loadFromFile`（点云） | `.ply` 且 `plyFileHasTriangleFaces` 为真时**拒绝**并提示改走网格导入（防止 Job 异步路径误当纯点云） |

位姿/颜色/属性：`hasPoseProperty` 等均为 `true`。

### 4.2 `MeshBackendData`

| 注册名 | `className()` = **`"Model"`**（显示名 Mesh） |
|--------|-----------------------------------------------|

| 方法 | 说明 |
|------|------|
| `setTriangleSoup` / `triangleSoup()` | 每三角 9 float（v0,v1,v2 各 xyz） |
| `setTriangleSoupWithNormals` / `triangleVertexNormals()` / `hasTriangleVertexNormals()` | 可选每顶点法线（9 float/三角，与 soup 下标对齐）；OBJ 含 `vn` 时写入 |
| `transformVerticesColumnMajorHomogeneous4x4(colMajor16)` | 列主序 4×4 烘焙顶点（URDF 世界烘焙、配准等）；**同时**旋转 `triangleVertexNormals` |
| `setTransformPivotAtOrigin(true)` | 烘焙后枢轴在原点；外包络 `modelCenter` 仍可非零，**不参与** pose 分解 |
| `loadFromFile` | 见 §4.2.1 |
| `loadStepHierarchyFromFile` / `loadDxfHierarchyFromFile` | 静态，输出 `MeshHierarchyPart` 列表 |

### 4.2.1 网格文件导入（`loadFromFile`）

统一写入 `triangleSoup`；`clearGeometry()` 同时清空 soup 与法线缓冲。

| 扩展名 | 实现 | 绕序 / 法线 |
|--------|------|-------------|
| `.step` / `.stp` | OCCT `STEPControl_Reader` + `BRepMesh_IncrementalMesh` | `TopAbs_REVERSED` 面导出时交换三角 n2/n3，避免场景光照发黑 |
| `.obj`（含 `vn`） | 自研解析 `v` / `vn` / `f v/vt/vn` | **保留文件顶点顺序**；`vn` 写入 `triangleVertexNormals`，供 `MeshBackendVisual` 光照（CGAL `read_polygon_soup` 会丢弃 `vn`） |
| `.obj`（无 `vn`） | CGAL `read_polygon_soup` | `orient_polygon_soup` 后保持绕序一致；封闭体按有符号体积整体翻转（避免逐三角质心翻转导致非凸面发黑） |
| `.stl` / `.ply` / `.off` | CGAL `read_polygon_soup` | 同 `.obj` 无 `vn` 路径（见下） |

**CGAL 无 `vn` 路径（`MeshBackendData_load_common.cpp`）**

1. `CGAL::IO::read_polygon_soup` → `orient_polygon_soup`（绕序一致化）。
2. `meshBuildSoupFromPolygons`：按 polygon 扇形三角化写入 `triangleSoup`，**不**做逐三角质心外向翻转。
3. `meshOrientSoupOutwardIfClosed`：以顶点质心为参考计算有符号体积；若 `vol` 相对包围盒尺度明显为负，则**整体**交换所有三角的 p1/p2；开放薄片（\|vol\| 近零）不翻转。

逐三角「相对质心朝外」翻转在非凸封闭体上会导致相邻面绕序不一致 → 场景光照下**部分面发黑**（已移除）。光照侧见 [`BackendVisual/DEVELOPER_GUIDE.md`](../../UI/BackendVisual/DEVELOPER_GUIDE.md) §4.2。

**现象与约定**：`BackendVisual` 在 `useSceneLighting=true` 时，无法线缓冲则用法线 `n = (p1-p0)×(p2-p0)`。绕序局部不一致会导致**部分或整面发黑**；带 `vn` 的 CAD/Max OBJ 必须走文件法线路径。

**Widget 导入**：STEP 优先 `BrepBackendData::loadStepHierarchyFromFile`（B-rep 装配，见 §4.4）；单件 STEP/BREP 走 `BrepBackendData::loadFromStepFile` / `loadFromBrepFile`。网格路径仍可用 `MeshBackendData::loadStepHierarchyFromFile`（tessellated 分件）。`.obj` 等不走 OSG `importModelFile` fallback（见该控制器注释）。

### 4.3 `struct MeshHierarchyPart`

STEP/DXF **mesh** 层级导入中间结构：`partPath`, `parentPartPath`, `displayName`, `triangleSoup`。

DXF 分件经 `dxfExpandInsertRecursive` 写入的 `triangleSoup` 为 **世界绝对坐标**；导入时 `pose=0`，**不做** Follow 求解（见 [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) §4.4.1a）。

### 4.4 `BrepBackendData`

| 注册名 | `className()` = **`"BrepModel"`**（catalog `BrepModel`） |
|--------|----------------------------------------------------------|

| 方法 | 说明 |
|------|------|
| `setShape` / `shapeRef()` | 持有 `geoalgo::ShapeHandle`；显示与特征离散共用 |
| `shareShapeFrom(other)` | 装配子零件共享 assembly shape |
| `loadFromStepFile` / `loadFromBrepFile` / `writeBrepFile` | STEP/BREP 文件 I/O |
| `loadStepHierarchyFromFile` | 静态；`collectShapeHierarchyTopology` → `BrepHierarchyPart[]`（**无 tessellation**） |
| `setBrepSidecarRelativePath` | 工程内嵌 `.brep` 相对路径 |

| `BrepHierarchyPart` 字段 | 说明 |
|-------------------------|------|
| `partPath` / `parentPartPath` | 装配树路径（与 mesh 层级相同约定） |
| `displayName` | 树节点显示名 |
| `shapeRef` | 当前实现为共享整件 assembly shape |

显示 tessellation 在 `GeometryAlgorithm::getOrBuildBrepImportArtifacts`（Phase1/2 分阶段）；Data 层只持久化 BREP shape + 工程 `.brep` sidecar，**不**持久化 display soup。

位姿/颜色/属性：`hasPoseProperty` 等均为 `true`。Visual 见 [`BackendVisual/DEVELOPER_GUIDE.md`](../../UI/BackendVisual/DEVELOPER_GUIDE.md) §4.3；Host 装配见 [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) §4.4.1b。

### 4.5 CAD 轨迹几何桥接（`GeometryRef.h` / `GeometryBackendOps.cpp`）

场景显示仍为 `MeshBackendData` 三角 soup；**特征离散**在运行时从 STEP 临时加载 B-rep（`geoalgo::readStepShape`），不经 Data 持久化 `TopoDS_Shape`。

| 类型 / API | 说明 |
|------------|------|
| `geometry_backend_ops::GeometryRef` | `backendIdUtf8` + `stepPathUtf8` + `frameId`；`stepPathUtf8` 与 `DocumentHost::backendSourcePath` 对齐 |
| `resolveGeometryRef` | 填充 `geoalgo::WorkpieceRef` |
| `discretizeFeature` / `discretizeFeatures` | 转发 `geoalgo::discretizeFeature` |
| `validateFeatureSpec` | 转发 `validateFeatureSpecWithShape`（含索引范围） |
| `enumerateFeatureCatalog` | 边/面目录，供 UI 与 LLM grounding |
| `featureSpecFromJson` / `featureSpecToJson` | JSON 契约 |
| `suggestFeaturesFromCatalog` | 规则启发式（焊/胶/磨意图关键词） |

UI 经 `IRobotDocumentHost::meshBackendStepSourcePath(backendId)` 解析 STEP 路径（`MainWindowRobotHost::DocumentHost` 转发 `DocumentPage::backendSourcePath()`）。

### 4.6 CAD 模板 + 扫描点云 B-rep 更新

**专题文档**：[`docs/template_brep_pointcloud_update.md`](../../../docs/template_brep_pointcloud_update.md)

| API（`geometry_backend_ops`） | 说明 |
|-------------------------------|------|
| `registerScanToCadTemplate` | 世界系 soup ICP；输出 `icpDeltaWorld`、`registrationPreviewOk`、`icpRmseGatePassed` |
| `updateBrepFromAlignedScan` | 原始 STEP + `scanPointsToTemplateModelFrame` → `updateShapeFromPointCloud` → `brepOut`；**不**注册场景 |
| `updateBrepFromCadTemplate` | 上述两步合并（单次调用场景） |

配准实现于 `GeometryBackendOps.cpp`：`runCoarseAlignmentPipeline` 编排粗配（`coarseStage=modeSelect/bbox/pca/frameCheck/ransac/soupMulti/coarseIcpLadder/soupRefine`）、`resolveRegistrationAlignMode`（**`pairHits=0` → `autoRecover`**，仅 `pairHits∈[1,31]` 且 maxDev 适中才 `manualPartial`）、`runReverseSoupMultiStageIcp`、**coarse ICP ladder 回退**（soup rollback 或 post-soup overlap 不足）。重叠度量与 PCA 评分使用 `KdTreePointSet` 加速。合成自检：`registrationCoarsePipelineSelfTest`（`pairHits=0 → autoRecover`、ladder maxPair 升序）。

配准在 Data 层将扫描/模板 soup 变换到世界系（`worldMatrix`）后 ICP；面归属前 `scanPointsToTemplateModelFrame` 变到模板文件系。

面更新算法见 [`docs/template_brep_pointcloud_update.md`](../../../docs/template_brep_pointcloud_update.md) §3。

| `TemplateBrepUpdateParams`（常用） | 说明 |
|-----------------------------------|------|
| `faceBandMm` / `normalThresholdDeg` | 面归属带与精 ICP 法线门控 |
| `maxAssignPointsPerFace` | 每面归属点预算（全工件自动摊薄） |
| `selectedFaceIndices` | 空=全工件；非空=选择性面 |
| `bsplineUvGridCellsU/V`、`bsplinePoleSmoothPasses` | BSpline UV 聚合与极点平滑 |

`TemplateBrepUpdateResult::skippedBadBboxFaceCount`：因单面或试应用全局 bbox 守卫而跳过的面数。

插件经 `PluginPointCloudHostImpl` 分步调用；面重构成功后 `registerAdoptedBrepAndLoadScene` + `alignFaceUpdatedBrepWithTemplateVisual` 注册**新** `BrepModel`（模板保留）。见 [`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md) §3.7 与专题 §2.2。

### 4.7 管状铸件特征构建（1.15.0+）

`geometry_backend_ops` 转发 `geoalgo::TubularGrinding*`（[`GeometryBackendOps.cpp`](source/GeometryBackendOps.cpp) / [`GeometryBackendOps.h`](../inc/GeometryBackendOps.h)）：

| API | 说明 |
|-----|------|
| `createTubularGrindingSession` | soup 快照 → `TubularGrindingSessionPtr` |
| `runTubularGrindingStage` | 单阶段执行（Segment / Centerline / TemplatePoints / Project） |
| `buildTubularGrindingSegmentColoredMeshSoup` 等 | 管段/环着色 mesh、环心点云、法向轴线、中心线/模板/投影点云 |

宿主 `PluginPointCloudHostImpl` 在 Segment 完成后注册 `_管段着色` / `_环着色` / `_环圆心` / `_法向` 等临时 `MeshBackendData`。算法与参数见 [`GeometryAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.5；UI 见 [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../../Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md)。

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
| `PointCloudAlgorithm` | 经 `PointCloudBackendOps` 做点云下采样/裁剪/配准/重建等（见 [`../PointCloudAlgorithm/DEVELOPER_GUIDE.md`](../PointCloudAlgorithm/DEVELOPER_GUIDE.md)）；插件经 SDK `IPluginPointCloudHost` 间接调用 |
| `VcgAlgorithms` | 经 `PointCloudBackendOps` 做网格后处理：简化/平滑/修复/重网格（运行时加载 `VcgAlgorithms.dll`；见 [`../VcgAlgorithms/DEVELOPER_GUIDE.md`](../VcgAlgorithms/DEVELOPER_GUIDE.md)） |
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
