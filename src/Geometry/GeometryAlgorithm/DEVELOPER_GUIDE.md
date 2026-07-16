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
| `FeatureSpec.h` | **CAD 轨迹特征 v2**：`FeatureListDocument` / `RawPath` / `FeatureCatalog`；见 `FeatureDiscretizerBridge.h` |
| `MeshTrajectory.h` | **Mesh 轨迹**：截面法平面求交、B 样条区域拟合；见 §3.2 |
| `TemplateBrepUpdate.h` | **扫描驱动 B-rep 更新**：面采样、点面归属、特征类型驱动面调整（Plane/Cylinder/Cone/Sphere/Toroid/BSpline） |
| `SelfTest.h` | `runSelfTest` |

### 网格离散模式（`MeshDiscretizeMode`）

- `AdaptiveTriangulation`：默认 STEP 路径（`BRepMesh_IncrementalMesh`）
- `UniformRelative` / `UVStructuredGrid`：质量与 UV 结构化
- `WireTubeMesh` / `WireRibbonMesh`：折线扫掠
- `RemeshSoup` / `PointCloudSurface`：预留（当前构建返回未实现）

### 密度控制（`MeshDensityControl`）

`MeshDiscretizeParams` 在质量预设之外提供互斥模式：

| `densityControl` | 行为 |
|------------------|------|
| `QualityPreset` | `applyQualityPreset` → Coarse/Medium/Fine |
| `TargetEdgeLength` | OCC deflection=`target×0.25`、角偏差=1°；`refine` 预加密到 `1.5×target`（≤40 万面）；Data 再 `isotropicRemesh` |
| `TargetTriangleCount` | 对相对 deflection 二分（约 8 次，相对容差 ±15%） |

边长均匀化不在本 DLL；Data `discretizeStepToMesh` 可选单次 `vcgalgo::isotropicRemesh`。勿用偏粗 OCC 基网格或 progressive remesh。

## 3.1 CAD 轨迹特征离散（v2 策略框架）

**设计原则**：对机器人执行层只有轨迹；工艺不在本 DLL 区分。离散策略通过 **Registry + 外置 JSON** 注册（对齐 HPLTPStrategy / TrajectoryAlgorithmBuiltins），**无 `FeatureKind` switch**。

| API（`FeatureDiscretizerBridge.h`） | 职责 |
|-----|------|
| `ensureFeatureDiscretizersRegistered` | 加载全部内置离散策略 |
| `discretizeFeatureList` | **唯一离散入口**；Coordinator 按 mergePolicy 合并 |
| `featureListFromJson` / `featureListToJson` | v2 文档 JSON |
| `validateFeatureListDocument` | 结构 + 策略校验 |
| `enumerateFeatureCatalog` / `suggestFeaturesFromCatalog` | 候选目录 + 意图启发式 |

**v2 文档**（`FeatureListDocument`，schemaVersion=2）：

```json
{
  "schemaVersion": 2,
  "workpiece": { "backendIdUtf8": "...", "stepPathUtf8": "..." },
  "defaultStrategyId": "EdgeChain",
  "features": [
    {
      "featureId": "edge_001",
      "strategyId": "EdgeChain",
      "geometry": { "edgeIndices": [3, 7] },
      "params": { "stepMm": 2.0, "linearDeflectionMm": 0.01 }
    }
  ]
}
```

| strategyId | mergePolicy | 说明 |
|------------|-------------|------|
| `EdgeChain` | LineConnectivity | 相连边合并 wire；不相连分量分段后拼接 |
| `FaceBoundary` / `FaceSection` / `FaceParamSurface` | FaceUnion | 多面 fuse 后离散 |
| `FaceIntersection` / `FaceOffsetCurve` / `SyntheticPolyline` | None | 逐行离散后拼接 |

外置参数：`resource/feature/discretizers/<StrategyId>.json`（PostBuild 拷贝至 `bin/x64(d)/resource/feature/`）。

实现：`discretizers/*Discretizer.cpp` + `FeatureDiscretizeCoordinator.cpp` + `FaceSectionDiscretize.cpp` / `ParamSurfaceDiscretize.cpp`。

**Breaking**：v1 `FeatureSpec` / `FeatureKind` / `discretizeFeature` 已移除。

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

### 3.1a Mesh 轨迹（`MeshTrajectory.h`）

**定位**：在 **三角 soup（模型坐标 mm）** 上生成机器人原始路径，与 CAD `FeatureSpec` 并列；入口 `generateMeshTrajectory`，输出 `RawPath`（可含多段 `segmentEndExclusive`）。

**坐标**：输入 soup 与 `MeshTrajectoryCrossSection` 均为 **mesh 模型系**；Host 预览/程序发射时乘 backend `worldMatrix`（见 RobotWidget `FeaturePickTransform`）。

#### 方法对比

| | **截面法** `CrossSection` | **B 样条区域** `BsplineRegion` |
|---|---------------------------|--------------------------------|
| 输入 | 平面原点 + 法向；可选 `region.triangleIndices` 过滤求交三角面 | 必选 ≥3 个选中三角面 |
| 几何 | 平面 ∩ 三角网格 → 多条折线 | 区域投影拟合 `Geom_BSplineSurface` → 参数域 UV 采样 |
| 离散 | 每条交线按 `discretize.stepMm` 弧长重采样 | **不用** `stepMm`；密度由 `uvCountU/V`、`fitUvSpacingMm` 与 `traceMode` 决定 |
| 输出段 | 每条交线一段，`RawPath.segmentEndExclusive` 标记分界 | 单段（蛇形/栅格顺序写入 `points`） |

#### 截面法流水线

```text
intersectPlaneWithTriangleSoup(soup, origin, normal, triFilter?)
  → 每三角最多 1 线段 → chainSegmentsToPolylines
  → discretizeAllMeshTrajectoryPolylines（按弧长降序，逐条 discretizeMeshTrajectoryPolyline）
  → RawPath.points + segmentEndExclusive[]
```

| API | 说明 |
|-----|------|
| `intersectPlaneWithTriangleSoup` | 平面-三角求交；`triangleIndexFilter` 非空时仅处理选中三角（交线在选区边界会断开） |
| `discretizeMeshTrajectoryPolyline` | 单折线 `stepMm` 重采样；法向默认取截面法向 |
| `generateMeshTrajectory` | 按 `spec.method` 分派；截面法走上述链路 |

**注意**：不再只保留最长交线；**所有**连通交线段均离散并追加。段间切向在 `importRawPathToTrajectory` 时不跨段取下一点。

#### B 样条区域流水线

```text
buildRegionFrame（选中三角质心 + 平均法向 → 区域 UV 系）
  → 投影网格：每个 (u,v) 格点 projectGridPointToRegion（最近三角面投影）
  → NurbsSurfaceFitting::fitNurbsSurfaceFromGrid（与曲面重构同源 AMRTO centripetal 最小二乘）
  → 参数域 [uMin,uMax]×[vMin,vMax] 均匀 outU×outV 采样（sampleSurfaceAtUv）
  → appendTraceModePoints（USerpentine / VSerpentine / UvGrid）
```

| 字段 | 说明 |
|------|------|
| `bspline.uvCountU/V` | 拟合采样格网基数（≥4，可被 `fitUvSpacingMm` 收紧） |
| `bspline.gridAngleDeg` | 区域 UV 系内扫描方向旋转 |
| `bspline.fitUvSpacingMm` | >0 时按区域 UV 跨度自动增加格点数（上限为 uvCount） |
| `bspline.traceMode` | JSON：`USerpentine` / `VSerpentine` / `UvGrid` |
| `bspline.fitMode` | 与 `MeshSurfaceNurbsFitMode` 同枚举；默认 `ApproxCentripetalFixedCtrlpts`（centripetal + 指定控制点） |
| `bspline.controlPointDensityFactor` | 控制点密度系数（默认 0.5，对齐曲面重构 AMRTO） |
| `bspline.nurbsDegreeU/V` | B 样条阶数（默认 3） |

| API | 说明 |
|-----|------|
| `buildBsplineRegionSurfacePreview` | 拟合同一曲面并三角化（16~48 细分），供 OSG 半透明预览 |
| `meshTrajectorySpecFromJson` / `meshTrajectorySpecToJson` | UI / PathPlan 持久化 |

实现：`source/MeshTrajectory.cpp`；SelfTest：`meshTrajectoryCrossSection`（立方体截面）、B 样条 fan 区域。

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

### 3.5 管状铸件特征构建（`TubularGrinding.h`，1.15.0+）

三角 soup（mm）→ **中心线** → 模板理想点位 → 表面投影。会话式 API 对齐曲面重构。

**当前 UI 流水线**（`PointCloudPlugin/TubularGrindingDockWidget`）：`None → Centerline → TemplatePoints → Project`。Segment 阶段已从 UI 移除，但 **API 与 `runPipeSegmentation` 仍保留**，自检 `SelfTest` 仍走四阶段。

| `TubularGrindingStage` | 源文件 | 说明 |
|------------------------|--------|------|
| Segment（可选 / 自检） | `TubularGrinding/PipeSegmentation.cpp` | 法向汇聚 → 环心 DBSCAN → 环链合并 → `TubularPipeSegment`；**Y/T 歧管多管分割** |
| Centerline | `TubularGrinding/CenterlineExtraction.cpp` + `TubularGrindingCommon.cpp` | **Laplacian 收缩骨架** → 全局 PCA 质心分箱 / 最长路径 → 弧长重采样 + Frenet 标架 |
| TemplatePoints | `TubularGrinding/TrajectoryTemplates.cpp` | 螺旋/环形/轴向/锯齿 + Auto 策略（按 `pipeId` 分组，依赖 `segments`） |
| Project | `TubularGrinding/MeshProjection.cpp` | 沿 ±模板法向投影；**网格**走射线-三角求交，**点云**走 Kd-tree top-K 最近邻（queryK=200，`runPointCloudProjection`） |

---

#### 中心线提取：Laplacian 收缩骨架（当前默认）

入口：`runCenterlineExtraction` → `runLaplacianSkeletonCenterline`（`TubularGrindingCommon.cpp`）。  
**设计目标**：在**不假设圆柱、不做管段分割**的前提下，从三角 mesh 直接提取一条中心折线，供后续模板点与投影使用。

##### 总体数据流

```text
buildIndexedMeshLite（量化焊接顶点 + 面邻接）
  → buildSkeletonGraphFromMesh（positions / anchors / faces / edges）
  → 迭代 centerlineIterations 次：
       contractSkeletonGraphStep（Cao 式拉普拉斯收缩 + 锚定）
       collapseAllBelowLength（短边塌缩）
       removeDegenerateFaces（退化面剔除）
  → 最终塌缩 + pruneShortLeafBranches（短叶枝剪枝）
  → extractCenterlineBySliceCentroids（主）或 extractLongestPathPolyline（备）
  → resamplePolylineToSamples（按 sectionSpacingMm 弧长重采样）
  → buildFrenetFrames（切向 / 法向 / 副法向）
```

##### 1. 骨架图 `SkeletonGraph`

从 `IndexedMeshLite` 构建，与原始 mesh 共享拓扑：

| 字段 | 含义 |
|------|------|
| `positions` | 当前顶点坐标（收缩过程中不断更新） |
| `anchors` | 锚定点（初始 = 原始表面顶点；中后期同步为收缩后位置，避免被拉回表面） |
| `faces` | 三角面顶点索引 |
| `edges` / `adjacency` | 无向边与邻接表 |

顶点经 **坐标量化焊接**（`buildIndexedMeshLite`，scale=1000），避免 soup 展开后邻接断裂。

##### 2. Cao 式拉普拉斯收缩（`contractSkeletonGraphStep`）

对每个顶点 \(v_i\)，设邻域 \(\mathcal{N}(i)\)，锚定权重 \(w\)（由 `computeContractionAnchorWeight` 调度）：

\[
v_i \leftarrow \frac{\sum_{j \in \mathcal{N}(i)} v_j + w \cdot a_i}{|\mathcal{N}(i)| + w}
\]

其中 \(a_i\) 为 `anchors[i]`。  
\(w\) 大 → 更贴近锚点（前期稳、贴壳）；\(w\) 小 → 更贴近邻域质心（后期向骨架收缩）。

**权重调度**（`computeContractionAnchorWeight`）：

| 阶段 | 迭代区间 | 行为 |
|------|----------|------|
| 爬升 | 前 60% | \(w\) 从 `weightStart` 对数增至 `weightPeak` |
| 释放 | 后 40% | \(w\) 线性降至 0 |

映射关系：

- `weightStart = max(1, laplacianAttraction × 5)`
- `weightPeak = clamp(laplacianLambda × 500, 10, 200)`

当 \(w \le 0.25 \times weightPeak\) 时，执行 `anchors = positions`，使锚点跟随收缩位置，**避免末期被拉回原始外表面**。

##### 3. 拓扑塌缩与剪枝

每轮迭代后：

- **边塌缩** `collapseAllBelowLength`：合并长度 < 阈值的边，阈值随迭代进度与 `bboxDiag`、平均边长成比例增大。
- **退化面剔除** `removeDegenerateFaces`：剔除面积或边长过小的三角面。

迭代结束后 **最终塌缩**（`finalCollapse = max(bboxDiag×0.02, avgEdge×0.85)`，最多 128 pass）+ **短叶枝剪枝** `pruneShortLeafBranches`（叶边长度 < `bboxDiag×0.02` 的 degree=1 顶点被移除）。

> **注意**：短枝剪枝 + 单路径提取，使当前实现**天然偏向单管**，Y/T 歧管支路会被削弱或忽略（见下文「已知限制」）。

##### 4. 中心折线提取（Stage C）

**主路径 — 全局 PCA 质心分箱**（`extractCenterlineBySliceCentroids`）：

1. 对收缩后全体顶点求质心 \(\bar{p}\) 与协方差主方向 \(\mathbf{u}\)（幂迭代，`computePrincipalAxisFromPoints`）。
2. 将各点标量投影 \(t_i = (\mathbf{p}_i - \bar{p}) \cdot \mathbf{u}\)，按 \(t\) 排序。
3. 以宽度 `sectionSpacingMm` 分箱，每箱内 3D 坐标算术平均 → 一个质心点。
4. 质心序列即中心折线（至少 2 个有效箱）。

**适用**：直管、弱弯管；点云沿单一主方向展开时效果较好。

**失效**：U 型弯 / 强分支时，全局 \(\mathbf{u}\) 近似「弦方向」，分箱平面斜切管壁，质心偏移；或收缩后沿主轴跨度 < `sectionSpacingMm/2` 导致函数返回 false。

**备路径 — 收缩图最长路径**（`extractLongestPathPolyline`）：

1. 在 `adjacency` 上从任意种子 BFS 找最远点 A，再从 A BFS 找最远点 B（图直径近似）。
2. Dijkstra(A → B) 得顶点路径，取 `positions` 序列。

仅在 PCA 分箱失败时启用；仍只输出 **一条** 路径，且路径沿图边，可能锯齿、贴壳。

##### 5. 重采样与标架

- `resamplePolylineToSamples`：沿折线弧长每 `sectionSpacingMm` 插入 `TubularCenterlineSample`（当前 `pipeId` 恒为 0，`radiusMm` 占位 1.0）。
- `buildFrenetFrames`：逐点构造 Frenet 标架（切向 / 法向 / 副法向），供模板点生成使用。

**成功条件**：`outSamples.size() >= 2`。若折线物理长度 < `sectionSpacingMm`，重采样可能只剩 1 点 → 整体失败（报错 `laplacian skeleton centerline extraction failed`）。

##### 6. 中心线相关参数

| 字段 | 默认 | 含义 |
|------|------|------|
| `centerlineIterations` | 80 | Laplacian 收缩 + 塌缩迭代次数 |
| `laplacianLambda` | 0.1 | 映射为中期锚定峰值权重（越大收缩越快） |
| `laplacianAttraction` | 0.2 | 初始锚定强度（越大前期越贴原网格） |
| `laplacianKNeighbors` | 8 | 保留字段；当前骨架图使用 **mesh 边邻接**，非 KNN |
| `sectionSpacingMm` | 2.0 | PCA 分箱宽度 + 输出中心线采样间距（mm） |
| `centerlineConvergenceEpsMm` | 0.01 | 已废弃，保留兼容 |

**调参提示**：

| 现象 | 建议 |
|------|------|
| 中心线贴外表面 | 略增 `laplacianLambda` 或迭代次数；确认锚点同步逻辑已生效 |
| 提取失败 / 塌没 | 降低 `centerlineIterations` 或 `laplacianLambda`；减小 `sectionSpacingMm`（短管） |
| 弯头锯齿 | 全局 PCA 固有限制；需局部切平面或分段中心线（未默认启用） |
| Y/T 只出一条臂 | 算法为单折线；需 Segment 多管或骨架树（见限制） |

##### 7. 可视化（Host 层）

中心线不以点云展示，而注册为 **overlay 线段**（`registerTubularGrindingCenterlineLines`）：

- `buildTubularGrindingCenterlinePolylineXyz` → 连续 `3×N` float
- 转为 `GL_LINES` 段，`overlayLinesAlwaysOnTop` + 深度 ALWAYS，避免被 mesh 遮挡

**PCA 主轴箭头**（`buildCenterlinePcaAxisArrowLineSegments` / `registerTubularGrindingPcaAxisArrowLines`）：

- 对 Laplacian 收缩后点云计算全局 PCA（与质心分箱相同）
- 箭头沿主方向从 `extentMin` 到 `extentMax`，箭头尖在 `extentMax` 端（绿色 overlay）

---

#### 管段分割（环分割，Segment 阶段 — 可选）

实现：`runPipeSegmentation`（`PipeSegmentation.cpp`）；网格预处理与射线汇聚在 `TubularGrindingCommon.cpp`。  
**与中心线的关系**：Segment 产出 `TubularPipeSegment[]` 与交汇面标记，是 **Y/T 歧管多 `pipeId` 中心线** 的既有路径；UI 默认跳过，导致 `TemplatePoints` 阶段 `segments` 常为空，仅 `pipeId=0` 的单管模板可工作。

```text
buildIndexedMeshLite（坐标量化焊接顶点 → faceNeighbors）
  → orientMeshFaceNormals（BFS 法向一致化）
  → 逐面 2-hop 向内射线 → approximateRayBundleCenter（放松汇聚，非严格交点）
  → 有效面环心 DBSCAN（ringCenterClusterEpsMm）
  → 簇内 mesh 连通性拆环 → TubularCrossSectionRing
  → 环邻接图 + 交汇环判定（junctionAxisSpreadDeg）
  → Union-Find 环链合并（axisMergeAngleDeg）→ TubularPipeSegment
```

| 步骤 | 函数 / 要点 |
|------|-------------|
| 邻接建图 | `buildIndexedMeshLite`：顶点坐标量化（scale=1000）焊接，避免 soup 展开后邻接断裂 |
| 法向一致化 | `orientMeshFaceNormals` |
| 环心估计 | `computeFaceInwardCenter` → `collectInwardRaySamples`（2-hop）→ `approximateRayBundleCenter` |
| 汇聚容差 | `ringRayConvergenceEpsMm`；0 时自动 ≈ `max(3mm, 局部跨度×55%)` |
| 环聚类 | `runDbscan` on 环心；`ringCenterClusterEpsMm` 为 0 时按最近邻距离自动估计 |
| 拆环 | `splitClusterByConnectivity`；`minRingFaces` 过滤过小环 |
| 交汇面 | 邻接环 ≥ 3 且轴线散布 > `junctionAxisSpreadDeg` → `kFaceJunction` |
| 管段合并 | `mergeRingsIntoSegments`；`minSegmentFaces` 不足时自动放宽一次 |

**Segment 主要参数**：

| 字段 | 默认 | 含义 |
|------|------|------|
| `ringCenterClusterEpsMm` | 0（自动） | 环心 DBSCAN 半径（mm） |
| `ringRayConvergenceEpsMm` | 0（自动） | 法向射线汇聚容差（mm） |
| `minRingFaces` | 4 | 有效环最少面数 |
| `axisMergeAngleDeg` | 28 | 相邻环轴线夹角上限（°），环链合并 |
| `junctionAxisSpreadDeg` | 38 | 三通交汇判定轴线散布（°） |
| `minSegmentFaces` | 40 | 有效管段最少面数 |
| `faceNormalAxisLengthMm` | 0（自动） | 法向轴可视化长度（mm） |
| `regionGrowAxisAngleDeg` | 28 | 保留兼容，环分割未使用 |

**广义截面分析**（Segment 内 `NeighborhoodMode::Adaptive` 路径，`analyzeCrossSection`）：在局部切平面上投影面心，椭圆/凸包拟合得环心与半径，与中心线阶段的「全局 PCA 分箱」相互独立。

---

#### 已知限制与失败模式

| 限制 | 说明 |
|------|------|
| **单折线输出** | 仅一条 `polyline`，无法表达 Y/T 树形中心线 |
| **全局 PCA** | 弯管/歧管上质心分箱可能斜切管壁；U 型弯尤甚 |
| **最长路径兜底** | 只覆盖图上一支，沿边锯齿 |
| **短枝剪枝** | 削弱歧管支路，利于单管、不利多分支 |
| **Template 依赖 Segment** | `runTrajectoryTemplates` 按 `segments` × `pipeId` 生成；无 Segment 时模板阶段易空或仅单管 |
| **统一错误文案** | Laplacian 失败时 `"laplacian skeleton centerline extraction failed"`；OTLC 失败时 `"otlc centerline polyline extraction failed"`，不区分建图 / 提线 / 重采样 |
| **点云仅 OTLC** | 点云输入只能用 `TubularGrindingCenterlineMethod::OtLc`，Laplacian 路径返回错误 |
| **点云无 Segment** | 点云没有 mesh 拓扑，Segment 阶段不可用（`ensureMesh` 返回 false） |
| **点云投影 top-K** | 点云投影使用 Kd-tree top-200 最近邻（非全遍历），大点云性能优于网格射线求交 |

**OTLC 双源中心线（新增）**：

入口 `runOtLcSkeletonCenterline`（`OtLcSkeleton.cpp`），支持 Mesh / PointCloud 双输入。网格走 welding-edge 邻接图，点云走 KNN + 互惠 DKNN 滤波。**点云必须走 OTLC**；Laplacian 路径对点云返回 `laplacian centerline requires mesh input`。

##### OTLC 算法流程

```
buildOriginalFromInput（原始点集 + 邻接图）
  → voxelDownsamplePoints（体素滤波降采样，bboxDiag/(N×rate)^(1/3)）
  → 每个体素 centroid → Kd-tree 找最近原始点作为 sample（snapping，确保 sample 在原始点位置）
  → 初始化 OtSkeletonState（sample / original / mass / union-find / sampleTree）
  → emitIterationSnapshot：活跃根 + 收缩 original 子采样
  → 迭代快照 onIteration(snap) ← 记录根点与收缩点云
  → estimatePointCloudInwardNormals（局部 PCA + 指向质心）
  → 预处理阶段（otcPreSteps 次）：
       assignOriginalPointsToSamples（Kd-tree 最近邻聚类，始终重建树确保索引一致）
       updateSamplePositionsFromClusters（Sinkhorn OT 更新）
       refreshSampleMedialPositions（射线束融合）
       otcClusterMergeStep（距离门控合并）
  → OTLC 外循环（最多 otLcOuterMaxIters 次，至少 5 次）：
       [Laplacian 收缩] contractPointCloudInwardLc（KNN 邻域 + 向内步进）
       [刷新法向] estimatePointCloudInwardNormals
       [OT 循环] otcOuterLoops 次 OT 更新 + 聚类合并
       [稀疏合并] sparseMergeSampleRoots（目标根数 ≈ bboxDiag/sectionSpacing × 0.6，受 minRootsBySamples 保护）
       → emitIterationSnapshot(outer+1) ← 活跃根 + 收缩 original 子采样
  → 终止条件（outer ≥ 5 后）：
       1. sampleRootCount ≤ targetRoots（根数达标）
       2. !anyMerge（拓扑稳定，无合并）
  → 最终合并 + rebuildSampleGraphEdges（PCA 对齐 + KNN 补边 + 跨分量桥接）
  → collectActiveSampleRoots + refineRootPositionsInward（射线束精炼）
  → 三级折线提取兜底（降序尝试）：
       1. extractClusterOrderedPolyline（PCA 排序合并，弧弦比门控 ≤4.5）
       2. extractCenterlineFromOtSkeleton（根图最长路径，isSampleGraphUsable 门控）
       3. extractSliceCentroidPolyline（全局 PCA 分箱）
          → extractOrderedCenterlinePolyline（KNN 最长路径兜底）
  → resamplePolylineToSamples + buildFrenetFrames
```

| 阶段 | 说明 |
|------|------|
| 降采样 | `voxelDownsamplePoints`：体素滤波 → 每个 centroid 找最近原始点（snapping）；确定性的密度保持降采样 |
| 法向估计 | 局部 PCA（KNN 邻域）最小特征向量 → 指向全局质心为 inward |
| Laplacian 收缩 | `contractPointCloudInwardLc`：约束 Laplacian（固定 mask 点不动）+ 向内步进 |
| OT 分配 | `updateSamplePositionsFromClusters`：Sinkhorn 软分配（sample 为原始点，w=exp(0)≈1 正常） |
| 聚类合并 | `otcClusterMergeStep`：距离门控合并 + 贪婪合并 `sparseMergeSampleRoots` |
| 迭代终止 | 前 5 轮强制运行（`outer >= 5` 保护），之后根数 ≤ targetRoots 或 `!anyMerge` 时退出 |
| 图建边 | `rebuildSampleGraphEdges`：PCA 方向对齐 + 排序串联 + KNN 补边 + `bridgeSampleEdgeComponents` 保证连通 |
| 根点精炼 | `refineRootPositionsInward`：所属原始点的 inward 射线束汇聚 |
| 折线提取 | 3 级兜底，质量门控 `isCenterlinePolylineReasonable`（弧弦比 ≤ 4.5） |

OTLC 参数（`OtLcParams`，由 `buildOtLcParams` 从 `TubularGrindingParams` 映射）：

| 字段 | 默认 | 映射来源 | 说明 |
|------|------|----------|------|
| `otSampleRate` | 0.10 | `params.otSampleRate` | 体素降采样比例，公式 `bboxDiag/(N×rate)^(1/3)`（越小体素越大，sample 越少） |
| `otCostBeta` | 3.0 | `params.otCostBeta` | OT 代价距离指数（越大簇边界越硬） |
| `pointCloudKnnK` | 30 | `params.pointCloudKnnK` | 点云 KNN 邻域大小 |
| `otcPreSteps` | 3 | `params.otcPreSteps` | 预处理 OT+合并轮次 |
| `otcOuterLoops` | 3 | `params.otcOuterLoops` | 每轮外循环内 OT+合并次数 |
| `otLcOuterMaxIters` | 40 | `params.otLcOuterMaxIters` | OTLC 外循环上限 |
| `minRootsBySamples` | 0（自动） | `params.minRootsBySamples` | 根点合并下限；0=auto: min(sampleCount, max(15, sampleCount×0.05)) |
| `skelConvergenceEps` | 1e-4 | 固定 | OT 能量收敛阈值 |

**迭代快照**：`runOtLcSkeletonCenterline` 的 `OtLcIterationCallback` 每轮传出 `OtLcIterationSnapshot`（`samplePositions`=活跃根，`contractedPositions`=收缩 original 子采样）。Host 注册 `_迭代N`（根，绿/黄/红）与 `_迭代N_收缩`（蓝）。

**限制**：
- 体素降采样 + snapping 需要点云有足够密度以保证每个体素 centroid 附近存在原始点；稀疏点云可能导致 snap 后 sample 重叠或不足
- 点云需足够稠密以支持 KNN 邻接图建边；稀疏点云易导致根图碎裂 + 全局 PCA 兜底

---

#### 点云投影（Project 阶段 — 点云路径）

实现：`runPointCloudProjection`（`MeshProjection.cpp`）。入口在 `TubularGrindingSession::runTubularGrindingStage(Project)` 中根据 `inputKind` 分流。

```text
对每个模板点 tp：
  origin = tp.positionMm
  direction = ±tp.normalMm（正反双向）
  → Kd-tree 搜索 top-200 最近点（pclalgo::KdTreePointSet）：
       沿 direction 投影距离 ≤ projectionMaxDistMm
       垂直距离 ≤ 0.5 × projectionMaxDistMm
       保留最近点（分别记正/反方向最近）
  → 选取双向中距离最近者作为投影点位
  → hitRate = 命中模板数 / 总模板数
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `projectionMaxDistMm` | 10.0 | 最大搜索半径（mm）；沿法向 + 垂直偏离均受此限制 |

**与网格投影的区别**：
- 网格投影走射线-三角求交（`BRepExtrema_ShapeProximity`），精度高但需 mesh 拓扑
- 点云投影走 **Kd-tree top-K 最近邻**（queryK=200，非全遍历），大点云性能优于网格射线求交

**`runLaplacianSkeletonCenterline` 返回 false 的典型原因**：

1. `buildSkeletonGraphFromMesh`：mesh 无效或无边。
2. PCA 与最长路径均失败：收缩后点数 < 3、主轴跨度不足、图不连通。
3. `resamplePolylineToSamples` 后样本数 < 2：折线过短相对 `sectionSpacingMm`。

---

#### 后续方向（未默认实现）

| 方向 | 说明 |
|------|------|
| Segment + 分段中心线 | 恢复或内部调用 `runPipeSegmentation`，每管段独立提线，`pipeId` 区分，交汇点共点 |
| 骨架树提取 | 保留分支、在 junction（degree≥3）分叉，输出多段折线 |
| 局部切平面质心 | 以 guide 折线定切向，在**原 mesh 面心**上切片；需质量门控与平滑 guide |
| 树形 overlay 显示 | 分支间不连线，`buildCenterlinePolylineXyz` 按 branch 断开 |
| 深度学习分割 | PointNet++ 等面语义 → 替代/并联 Segment |

---

#### 可视化导出

| API | 用途 |
|-----|------|
| `buildSegmentColoredMeshSoup` | 按管段 HSV 着色 mesh |
| `buildRingColoredMeshSoup` | 按环着色（调试验证） |
| `buildRingCenterPointsCloud` | 环心点云 |
| `buildFaceNormalAxisLineSegments` | 每面法向轴线（6 float/段，供 overlay 线渲染） |
| `buildCenterlinePointsCloud` / `buildCenterlinePolylineXyz` | 中心线点云 / 折线 |
| `buildTemplatePointsCloud` / `buildProjectedPointsCloud` | 模板点 / 投影点 |

入口：`createTubularGrindingSession` → `runTubularGrindingStage`。

**Data 转发**：`geometry_backend_ops::createTubularGrindingSession`、`buildTubularGrindingRingColoredMeshSoup`、`buildTubularGrindingFaceNormalAxisLineSegments`、`buildTubularGrindingCenterlinePolylineXyz` 等（[`GeometryBackendOps.h`](../Data/inc/GeometryBackendOps.h)）。

**调参提示**：分割过碎时优先增大 `ringRayConvergenceEpsMm`（常见 8–25 mm）或 `ringCenterClusterEpsMm`；交汇误判可增大 `junctionAxisSpreadDeg`。中心线失败时优先降低收缩强度或 `sectionSpacingMm`。

**报告字段**（`TubularGrindingReport`）：`pipeCount`、`ringCount`、`junctionFaceCount`、`regionCountBeforeFilter`、`centerlinePointCount`、`sectionFitFailCount` 等。

自检：`SelfTest.cpp` 中 `tubularGrinding*`（OCCT 圆柱离散 → Segment + Centerline + Template + Project + 投影命中率门禁）。

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
- [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../../Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md) — 特征构建 UI 与调参
- [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)
- [`docs/mesh_surface_reconstruction.md`](../../docs/mesh_surface_reconstruction.md)
- [`RobotScene/DEVELOPER_GUIDE.md`](../../Robot/RobotScene/DEVELOPER_GUIDE.md) §14 — `RawTrajectory` 编辑流水线；`TubularGrindingTrajectoryIngress`（桩）
