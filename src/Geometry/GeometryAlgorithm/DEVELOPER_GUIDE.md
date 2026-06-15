# GeometryAlgorithm 模块开发文档

编码约定见 [`CONVENTIONS.md`](CONVENTIONS.md)。

## 1. 模块定位

`GeometryAlgorithm` 是 **CAD/网格几何算法 DLL**：OCC 点/线/面离散、网格离散、求交、B-rep 布尔、线/面融合；CGAL 网格布尔。不依赖 Qt/OSG。

| 属性 | 说明 |
|------|------|
| 输出 | x64 `GeometryAlgorithm.dll` → `bin/x64(d)/` |
| 命名空间 | `geoalgo` |
| 依赖 | OCCT 7.9、CGAL 5.5.2、Boost、Eigen |

## 2. 数据契约

**Breaking v2**：本 DLL **不持有 `worldMatrix`**。调用方（Data/Host）负责将点云/shape 变换到同一坐标系后再调用；配准输出 `icpDeltaWorld` 由 Host 写入 `template.worldMatrix`。

| 类型 | 布局 |
|------|------|
| 折线 | `3*N` float（有序顶点，mm） |
| 三角 soup | `9*T` float（每三角 3 顶点 xyz） |
| STEP 路径 | 本地窄字节（`QFile::encodeName`） |

## 3. API 总览

| 头文件 | 功能 |
|--------|------|
| `Discretize.h` | Edge/Wire/Shape 折线与三角 soup；STEP 单件/层级 |
| `BrepImportArtifacts.h` | BREP 导入预处理：显示 soup、面/边离散、按 `ShapeHandle` 共享缓存 |
| `MeshDiscretize.h` | 自适应/UV 网格/线管带网格；质量预设 |
| `ShapeIo.h` / `ShapeQuery.h` | STEP 读入；按索引求交/离散编排 |
| `Intersection.h` | 线线、线面、面面、形体截面 |
| `BrepBoolean.h` | OCC Fuse/Common/Cut → Shape 或 mesh |
| `WireOps.h` / `ShellOps.h` | 线融合、面缝合 |
| `GeoMeshBoolean.h` | CGAL 三角网格布尔 |
| `FeatureSpec.h` | **CAD 轨迹特征**：`FeatureSpec` / `RawPath` / `FeatureCatalog`；统一离散入口 `discretizeFeature` |
| `TemplateBrepUpdate.h` | **扫描驱动 B-rep 更新**：面采样、点面归属、特征类型驱动面调整（Plane/Cylinder/Cone/Sphere/Toroid/BSpline） |
| `SelfTest.h` | `runSelfTest` |

### 网格离散模式（`MeshDiscretizeMode`）

- `AdaptiveTriangulation`：默认 STEP 路径（`BRepMesh_IncrementalMesh`）
- `UniformRelative` / `UVStructuredGrid`：质量与 UV 结构化
- `WireTubeMesh` / `WireRibbonMesh`：折线扫掠
- `RemeshSoup` / `PointCloudSurface`：预留（当前构建返回未实现）

## 3.1 CAD 轨迹特征离散（`FeatureSpec.h`）

**设计原则**：对机器人执行层只有轨迹；工艺（焊缝/涂胶/打磨）不在本 DLL 区分，仅通过下游 `RawTrajectory` 编辑配方体现。

| API | 职责 |
|-----|------|
| `validateFeatureSpec` / `validateFeatureSpecWithShape` | 结构校验；后者加载 STEP 检查边/面索引 |
| `discretizeFeature` | **唯一离散入口**；按 `FeatureKind` 内部分派 |
| `discretizeFeatures` | 批量离散 |
| `enumerateFeatureCatalog` | 边/面拓扑摘要 + 启发式标签（焊缝/涂胶/打磨候选） |
| `featureSpecFromJson` / `featureSpecToJson` | UI / AI / 配置文件共用 JSON |
| `suggestFeaturesFromCatalog` | 规则回退：按意图从目录生成 `FeatureSpec[]` |

| `FeatureKind` | 说明 |
|---------------|------|
| `EdgeChain` | 单边或有序边链 |
| `FaceBoundary` | 面外轮廓 |
| `FaceIntersection` | 两面交线 |
| `FaceOffsetCurve` | 面内边沿法向偏置 |
| `FaceUVGrid` | 面内 UV 扫描点族（打磨栅格） |
| `Composite` | 子特征顺序拼接 |
| `SyntheticPolyline` | 外部/LLM 点列透传（无 STEP 降级路径） |

实现：`source/FeatureDiscretize.cpp`；复用 `Discretize` / `Intersection` / `ShapeQuery` / `WireOps`，不对外暴露按类型的顶层 API。

**JSON 示例**（与计划文档一致）：

```json
{
  "schemaVersion": 1,
  "featureId": "seam_01",
  "workpiece": { "backendIdUtf8": "mesh_12", "stepPathUtf8": "D:/part.step" },
  "kind": "FaceIntersection",
  "refs": { "faceIndices": [3, 7] },
  "discretize": { "stepMm": 2.0, "outputTangent": true, "outputNormal": true }
}
```

### 3.2 BREP 导入预处理（`BrepImportArtifacts.h` / `Discretize.h`）

#### 离散算法（精度不变）

| 入口 | 说明 |
|------|------|
| `discretizeShapeToSoupPerFace(shape, params, soup, triFaceIndex, faceSoups?)` | **整件** `meshShapeIncremental` 一次（`InParallel=true`），再按 `shapeFaceAtIndex` 顺序逐面提取；替代旧版逐面重复 `BRepMesh` |
| `tessellateShapePerFaceMedium(shape, …)` | 固定 Medium：`linearDeflectionMm=0.01`、`angularDeflectionDeg=0.5`；内部调用 `discretizeShapeToSoupPerFace` |

特征离散（`discretizeFeature`）仍从精确 BREP/STEP 计算，**不依赖** display soup。

#### Artifacts API

| API | 说明 |
|-----|------|
| `buildBrepImportArtifactsDisplay(shape, out[, timings])` | **Phase1**：display soup + `triangleFaceIndex` + `faceSoups` + 预计算 `displayNormals` |
| `buildBrepImportArtifactsPick(shape, out[, timings])` | **Phase2**：`collectShapeFaceEdgeIndices` + 边线框（0.05mm / 1°） |
| `ensureBrepImportPickArtifacts(shape, artifacts)` | Phase2 懒构建（`pickReady` + mutex；bind/线框/拾取时） |
| `buildBrepImportArtifacts(shape, out)` | Phase1 + Phase2 全量（兼容旧调用） |
| `getOrBuildBrepImportArtifacts(shape[, err, timings])` | 按 `ShapeHandle::isSame()` 缓存 **Phase1**；返回 `shared_ptr<BrepImportArtifacts>` |
| `clearBrepImportArtifactsCache()` | 测试或工程切换时清空 |

内存缓存最多 **16** 个 shape，超出淘汰最早条目。RunLogger 分段耗时由 Host 在导入时输出（`BrepImportBuildTimings::meshMs` / `pickMs` / `triangleCount`）。

#### 异步导入编排（Host + Widget）

```text
Worker executeLoad
  ├─ 读 STEP/BREP
  └─ getOrBuildBrepImportArtifacts（仅 Phase1）
UI finishIntoDocument
  ├─ registerAdoptedBrep… + loadBackend（showWireOutline=false，useSceneLighting=true）
  └─ 树/相机/选中
Widget JobSystem（可选）
  └─ warmPickArtifacts → buildBrepImportArtifactsPick（Phase2 预热）
```

| `BrepImportArtifacts` 字段 | 用途 |
|---------------------------|------|
| `displaySoup` / `displayNormals` / `triangleFaceIndex` | 三角显示、光照法线、面 id 映射 |
| `faceSoups` | 每面局部 soup（BREP 面拾取/高亮） |
| `edgePolylines` / `faceEdgeIndices` | 边线框与面-边拓扑（Phase2） |
| `pickReady` | Phase2 是否已构建 |

**层级拓扑（无 tessellation）**：`collectShapeHierarchyTopology(shape, outParts)` 仅遍历 OCCT 装配树，输出 `MeshHierarchyPart` 路径/显示名；**不**填充 `triangleSoup`。Data 层 `BrepBackendData::loadStepHierarchyFromFile` 将其转为 `BrepHierarchyPart` 并为各零件共享同一 `assembly` `ShapeHandle`。

带 tessellation 的 `collectShapeHierarchy` 仍供网格路径（DXF/旧 STEP mesh 回退）使用。

### 3.3 模板 B-rep 面更新（`TemplateBrepUpdate.h`）

扫描点云（STEP 模型坐标 mm）驱动模板 shape 逐面调整。编排与 ICP 在 `Data/GeometryBackendOps.cpp`；**完整流程与守卫逻辑**见 [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md) §4。

| API | 说明 |
|-----|------|
| `sampleShapeSurfacePoints` | 按 `spacingMm` 在 `TopoDS_Face` 参数域网格采样 → `outXyz`（STEP 坐标） |
| `updateShapeFromPointCloud` | 并行点归属 → 并行逐面 `adjustFaceGeometryDispatch` → **增量试应用** → `updatedShape` |
| `TemplateBrepUpdateResult` | `perFace[]`、`updatedFaceCount`、`skippedBadBboxFaceCount`、`globalMaxDeviationMm`、`qualityPassed` |

**点归属**（`assignScanPointsParallel`）：面 bbox 预筛 + 单面投影，OpenMP 并行；全工件时 `effectiveAssignPointsPerFace` 按面数摊薄采样预算。

**增量写回**：每面 `BRepTools_ReShape::Apply` 试替换后检查整体 bbox；失败则跳过该面，避免批量 Apply 后包围盒爆炸。单面 bbox、ShapeFix 膨胀、最终全局 bbox 均有守卫（见专题文档 §4.4）。

**选择性面重构**：`selectedFaceIndices` 为空处理所有面；非空仅调整指定面。`qualityPassed` 在 selective 模式下仅统计已更新面归属点偏差。

**BSpline**：`bsplineAdjustThresholdMm`、`bsplineMaxPoleMoveMm`、`bsplineUvGridCellsU/V`、`bsplinePoleSmoothPasses`；超阈值点经 UV 网格聚合后 `SetPole`，保留原 wire。

**报告**：`FaceUpdateReport::surfaceTypeName` 为 OCCT 曲面类型名；`action` 见下表。

**面调整策略**（`adjustFaceGeometryDispatch`）：根据原始 CAD 面类型选择不同调整方式，而非统一拟合：

| 面类型（`GeomAbs_SurfaceType`） | 调整方式 | `FaceUpdateAction` |
|-------------------------------|----------|-------------------|
| `Plane` | 最小二乘拟合平面 + 原始 wire | `PlaneAdjusted` |
| `Cylinder` | PCA 轴线 + 径向均值半径，保留 UV 范围 | `CylinderAdjusted` |
| `Cone` | PCA 轴线 + 半角估算，保留 UV 范围 | `ConeAdjusted` |
| `Sphere` | 质心 + 径向均值半径，保留 UV 范围 | `SphereAdjusted` |
| `Torus` | PCA 轴线 + 中位数主半径 + 偏差次半径 | `ToroidAdjusted` |
| `BSplineSurface` | 扫描点投影原面；超 `bsplineAdjustThresholdMm` 的点经基函数加权调整控制点（`SetPole`），保留结点/wire | `BSplineAdjusted` |
| 其他 | 降级为 `refitFaceFromPoints`（全拟合） | `PlaneRefit` 等 |

| `FaceUpdateAction` | 含义 |
|--------------------|------|
| `PlaneAdjusted` ~ `BSplineAdjusted` | 特征类型驱动调整（新） |
| `PlaneRefit` / `CylinderRefit` / `FreeformRefit` | 全拟合（旧，Other 类型降级路径） |
| `ConeRefit` / `SphereRefit` / `ToroidRefit` | 预留拟合枚举 |
| `SkippedNoPoints` | 带内无足够扫描点 |
| `Unchanged` | 未改 |

### 3.4 网格曲面重构（`MeshSurfaceReconstruction.h`）

三角 soup（mm）→ 分块 B 样条（或平面回退）→ C² 混合 → 局部光顺 → `ShapeHandle`。预处理（Vcg 修复 → 各向同性重网格 → 法矢光顺）在 **Data** 层完成，可单独调用 `preprocessMeshSoupForSurfaceReconstruct`。

| `MeshSurfaceReconstructStage` | 源文件 | 说明 |
|-------------------------------|--------|------|
| Preprocess（Data 层） | `VcgAlgorithms/MeshRepair.*` + `MeshRemesh.*` + `MeshNormalSmooth.*` | 修复 → 均匀化（边长中位数/手动）→ Kuwahara + 法矢拉普拉斯 |
| Partition / Sample / Fit | `MeshSurfaceReconstruction/*` | O(F) 邻接建图 + 特征棱检测 + FPS 种子 + 质量感知合并；**AMRTO 调和 UV 栅格采样**（`amrto-harmonic` / harmonic / PCA 回退）+ **`NurbsSurfaceFitting` centripetal 最小二乘** + 平面回退；分 patch `fitRejectReason` 聚合 |
| BoundaryBlend / JunctionBlend | `BoundaryBlend.cpp` / `JunctionBlend.cpp` | Bezier 权混合（首版简化） |
| Fair | `BsplineSurfaceFairing.cpp` | Hahmann 式局部指标（首版简化） |
| Assemble | `MeshSurfaceReconstructionAssemble.cpp` | `TopoDS_Compound`（开放曲面不 Sewing） |
| 输出校验 | `MeshSurfaceReconstructionValidate.cpp` | 三角化非空、单面 ≤8000 三角、包围盒比例 ≤3 |

**会话式分阶段**（1.13.0+）：`createMeshSurfaceReconstructSession(soup)` → `runMeshSurfaceReconstructStage(session, stage, …)` 按序执行；`reconstructBrepFromMeshSoup` 内部复用同一会话流水线。

入口：`geoalgo::reconstructBrepFromMeshSoup`（全流程）或 `runMeshSurfaceReconstructStage`（单步）。自检：`SelfTest.cpp` 中 `meshSurfaceReconstruct`（盒体 soup）。

**栅格采样**（`PatchParameterize.cpp`）：

| 函数 / 组件 | 说明 |
|-------------|------|
| `buildPatchPcaFrame` | 顶点 + 面心扩展 PCA 切平面框 |
| `sampleUniformPhysicalGrid` | 默认：物理等距栅格 + `PatchClosestAccel` 曲面投影 |
| `solveHarmonicUv` / `sampleHarmonicFaceCentroidGrid` | 小 patch 可选调和 UV（≤3000 面，80 轮迭代） |
| `sampleCentroidAnchoredPcaGrid` | 质量门禁失败时的面心锚定回退 |
| `passesSampleQuality` | `diagRatio ≥ 0.75` 且 `unique ≥ 0.65` |
| `buildSamplePointsCloud` | 合并各 patch 采样点为场景点云 |

详见 [`docs/mesh_surface_reconstruction.md`](../../docs/mesh_surface_reconstruction.md)。

## 4. Data 薄包装

[`GeometryBackendOps.h`](../Data/inc/GeometryBackendOps.h)（`geometry_backend_ops`）转发 STEP 路径级 API，供 `CloudSimPluginHost` 调用。STEP 导入仍经 `MeshBackendData::loadFromFile` → `geoalgo::tessellateStepFile`。

**模板 B-rep 更新**另含 `registerScanToCadTemplate` / `updateBrepFromAlignedScan`（见 [`Data/DEVELOPER_GUIDE.md`](../Data/DEVELOPER_GUIDE.md) §4.6）。

特征轨迹 API 另见 [`GeometryRef.h`](../Data/inc/GeometryRef.h)：`resolveGeometryRef`、`discretizeFeature`、`enumerateFeatureCatalog` 等（`geometry_backend_ops` 命名空间）。

## 5. 插件 SDK（1.5.0+）

- `IPluginGeometryHost`：异步离散/求交/布尔（见 `CloudSimPluginSDK/inc/IPluginGeometryHost.h`）
- 宿主：`PluginGeometryHostImpl` → `DocumentGeometryOps` → `geometry_backend_ops`
- 示例：`plugins/com.cloudsim.geometry/GeometryPlugin`

## 6. 自检

```cpp
std::string err;
const bool ok = geoalgo::runSelfTest(&err);
```

## 7. 相关文档

- [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../../Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md)
- [`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)
- [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)
- [`docs/mesh_surface_reconstruction.md`](../../docs/mesh_surface_reconstruction.md)
- [`RobotScene/DEVELOPER_GUIDE.md`](../../Robot/RobotScene/DEVELOPER_GUIDE.md) §14 — `RawTrajectory` 编辑流水线
