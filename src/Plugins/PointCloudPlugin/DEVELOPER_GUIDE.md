# PointCloudPlugin 示例

点云处理插件，演示 **1.2.0+** SDK：`IPluginPointCloudHost` + 侧栏 UI。

## 构建与部署

| 项 | 说明 |
|----|------|
| 工程 | `PointCloudPlugin.vcxproj`（x64，v142，Qt 5.14.2） |
| 链接 | **仅** `CloudSimPluginSDK.lib` |
| 部署 | `bin/x64(d)/plugins/com.cloudsim.pointcloud/plugin.json` + `PointCloudPlugin.dll` |
| `minHostVersion` | `"1.17.0"`（SDF/DDF 非刚性配准 + SPARE + 分割标注宿主 API） |

## 运行时

- 侧栏 Tab **点云** / **Point Cloud**：导入、列表、下采样、裁剪（包围盒/球/多边形）、预处理、**ICP / SPARE / SDF·DDF 配准**、重建
- 侧栏 Tab **特征构建** / **Feature Build**（**1.15.0+**）：管状铸件 Phase 1–4 分阶段调试（见下节）
- 菜单 **Tools → Point Cloud**（中文下子菜单标题为 **点云**）
- 语言：默认中文；切换 **设置 → Language → 中文/English** 时侧栏与菜单同步更新
- **重建网格 → 导出 PLY**：侧栏「重建网格」区选网格对象，点 **导出 PLY…**（或菜单 **Tools → 点云 → 导出网格 PLY…`）

**多边形裁剪（1.11.0+）**（侧栏「裁剪」区）：

1. 选中点云 → 选择 **保留内部** / **删除内部**
2. **多边形裁剪…** → `pickPolylineFromViewport` 进入 3D 绘制（左键顶点、右键/双击闭合、Esc 取消）
3. 闭合后自动 `cropPointCloudByPolyline`（屏幕投影 + 点在多边形内判断）

典型流程：

1. `importFileIntoActiveDocument(path, true)` 得 `backendId`
2. `doc->queryPointCloudInfo(id, info)` 显示点数
3. `host->pointCloudHost()->downsamplePointCloudVoxel(doc, id, params, onFinished)`
4. `onFinished` 中刷新列表；场景由宿主自动 `loadPointCloudFromBackendData`

**模板 B-rep 更新**（侧栏「CAD 模板 B-rep 更新」区）：

1. 导入 STEP → `BrepModel`、扫描 PLY → 点云，或 Poisson/导入 → `Model` 网格
2. 3D 视图手动对齐点云/网格与 CAD
3. 侧栏「CAD 模板 B-rep 更新」：**扫描数据** 下拉选点云或网格；选择模板 B-rep；可选 **选择面…**（`geometryHost()->pickStepElementFromViewport`）累积面索引，空列表=全部面
4. **匹配 (ICP)** → `registerScanToCadTemplate`
5. **面重构** → `updateTemplateBrepFromAlignedScan`（须先匹配）；详见 [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)

## SPARE 非刚性配准（1.16.0+）

原理与调参通俗说明见 [`docs/spare_nonrigid_registration.md`](../../../docs/spare_nonrigid_registration.md)。

侧栏「配准」区：**方法** 下拉可选 **刚性 ICP**、**SPARE 非刚性**（**1.16.0+**）或 **SDF/DDF 非刚性**（**1.17.0+**）。

| 控件 | 说明 |
|------|------|
| 非刚性源 / 目标 | 独立下拉，列出点云与网格（SPARE 与 SDF 共用） |
| SPARE 选项 | 体素预滤波 (mm)、刚性预对齐、输出为新对象 |
| 执行 SPARE | `nonRigidRegisterSpare(...)` |

典型流程：

1. 方法选 **SPARE**；在「非刚性源 / 目标」下拉中分别选点云或网格（须为不同对象）
2. 可选体素预滤波、刚性 ICP 预对齐
3. 勾选「输出为新对象」则生成 `*_SPARE` / `*_spare` 后缀对象
4. 回调 `PluginPointCloudJobResult`：`rmseMm`（平均对称点-面误差）、`spareDeformationNodeCount`

算法与参数映射见 [`PointCloudAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/PointCloudAlgorithm/DEVELOPER_GUIDE.md) §3.5。

## SDF/DDF 混合非刚性配准（1.17.0+）

原理见 [`docs/sdf_nonrigid_registration.md`](../../../docs/sdf_nonrigid_registration.md)。

| 控件 | 说明 |
|------|------|
| 场模式 | DDF 有向距离 / 有符号 SDF |
| 场体素 (mm) | 0=自动 |
| 细阶段数据项 | 点-面（默认）/ DDF / SDF |
| 刚性预对齐、输出为新对象 | 同 SPARE |
| 执行 | `nonRigidRegisterSdf(doc, sourceBackendId, PluginPointCloudSdfParams, onFinished)` |

须与宿主同时升级到 **1.17.0+**（vtable）。算法见 PointCloudAlgorithm §3.6。

## 网格后处理（1.9.0+，需 VcgAlgorithms.dll）

侧栏「网格后处理」区提供基于 vcglib 的网格操作：

| 操作 | 说明 | 参数 |
|------|------|------|
| **网格简化** | quadric-edge-collapse 面数精简 | 目标面数、质量阈值 |
| **Laplacian 平滑** | 快速拉普拉斯平滑（余切权重、边界保护） | 迭代次数 |
| **Taubin 平滑** | λ/μ 保形平滑 | 迭代次数、λ |
| **网格修复** | 去重/退化/重复面/非流形/可选填孔 | 填孔开关、孔洞最大边数 |
| **各向同性重网格** | 均匀三角形分布 | 目标边长(mm) |

典型流程：

1. 重建或导入网格 → 网格出现在「网格对象」下拉
2. 选中网格 → 显示面数/顶点数
3. 调整参数 → 点击操作按钮
4. 结果作为新网格对象创建，自动选中

菜单入口：**Tools → 点云 → 网格简化 / 网格平滑**

宿主需链接 `VcgAlgorithms.dll`；未链接时操作返回错误提示。

## 曲面重构（1.12.0+ 全流程，1.13.0+ 分阶段）

侧栏「曲面重构」区（与「网格后处理」共用 `m_meshTargetCombo`）：

| 控件 | 参数 / API |
|------|------------|
| 法矢光顺迭代 / 特征阈值 c0 | `normalSmoothIterations` / `featureThresholdC0` |
| 分块数（0=自动）/ 分块法向平滑 / 特征百分位 | `patchCountHint` / `partitionNormalSmoothIters` / `featureAnglePercentile` |
| 每边采样 n（间距=0 时） | `samplesPerPatchEdge` |
| UV 目标间距 / 最少·最多每边 | `targetUvSpacingMm`（0=固定 n）/ `minSamplesPerEdge` / `maxSamplesPerEdge`（0=无上限） |
| 拟合最多/边 / 拟合 UV 间距 | `maxFitGridPerEdge`（默认 9，0=不降采样上限）/ `fitUvSpacingMm`（0=仅用最多/边） |
| 采样率 k / 控制点密度 | `sampleRateFactor`（默认 2.0）/ `controlPointDensityFactor`（默认 0.5，AMRTO k_sample/k_type_gemodl） |
| NURBS 拟合模式 | `fitMode`（默认最小二乘+指定控制点数） |
| 混合带宽度（0=自动）| `blendStripWidth`（边界/交汇混合阶段） |
| 分块算法 | `partitionMode`（v3 / Hybrid / **CgalChartHybrid**） |
| 调和边界模式 | `harmonicBoundaryMode`（默认 GeodesicSquare，AMRTO 测地 square-border） |
| 光顺 ε / 最大迭代 | `fairingEpsilon` / `fairingMaxIterations` |
| 装配离散精度 (mm) | `tessellateLinearDeflectionMm`（装配阶段 B-rep 三角化精度） |
| **预处理后写入场景网格** | `exportPreprocessedMeshToScene`（临时 `Model`，名 `源名_预处理后`） |
| **预处理 → 装配输出**（8 按钮） | `beginMeshSurfaceReconstructSession` + `runMeshSurfaceReconstructStage` |
| **全流程** | `reconstructSurfaceFromMesh`（一次跑完，兼容 1.12） |
| **重置会话** | `clearMeshSurfaceReconstructSession` |

分阶段顺序（须按序执行，切换网格对象自动重置会话）：

1. **预处理** — Vcg 修复 + 法矢光顺；可选写入场景网格
2. **分块** — 四边域划分
3. **栅格采样** — PCA 物理等距栅格 + 曲面投影；质量未过时调和 UV（小 patch）或面心锚定回退（`pca-centroid`）
4. **NURBS 拟合** — AMRTO 式 centripetal 最小二乘（`NurbsSurfaceFitting`）；摘要含 `fitRejectApprox/Pole/FitGrid/FullGrid/MakeFace` 拒因统计
5. **边界混合** — 片间边界 C²
6. **交汇混合** — 多片交汇 C²
7. **光顺** — B 样条局部光顺
8. **装配输出** — Compound 装配 → 新 `BrepModel`

每阶段中文摘要写入：侧栏日志区、状态栏、RunInfo `[曲面重构]`（`PluginMeshSurfaceReconstructReport::stageSummaryZh`）。

典型调试流程：

1. Poisson 重建或导入 → 选中 `Model` 网格
2. 勾选「预处理后写入场景网格」→ 点 **预处理**，检查 `*_预处理后` 网格
3. 依次点 **分块** … **装配输出**，观察侧栏日志与 RunInfo
4. 或直接用 **全流程** 一次完成（菜单入口仍走全流程）

菜单：**Tools → 点云 → 曲面重构**（触发全流程）

详见 [`docs/mesh_surface_reconstruction.md`](../../docs/mesh_surface_reconstruction.md)。

## 特征构建（1.15.0+，管状铸件打磨 MVP）

侧栏 **特征构建** Tab（`TubularGrindingDockWidget`），与「点云」并列；需宿主 **1.15.0+**。

**当前 UI 阶段顺序**：`None → Centerline → TemplatePoints → Project`（管段分割按钮已移除；Segment API 仍可用于自检或二次开发）。

| 按钮 | 阶段 | 场景对象 |
|------|------|----------|
| 中心线 | `Centerline` | `源名_中心线`（**红色 overlay 折线**）、`源名_PCA轴`（**绿色 PCA 主轴箭头**） |
| 模板点位 | `TemplatePoints` | `源名_模板点位`（点云） |
| 表面投影 | `Project` | `源名_投影点位`（点云） |
| 重置会话 | — | 清除上述全部临时对象 |

中心线算法分两套，由「提取算法」下拉切换：

| 算法 | 数据源支持 | 原理（摘要） |
|------|-----------|--------------|
| **Laplacian**（默认） | 仅 Mesh（三角网格） | 网格边 Laplacian 收缩 + 边塌缩 → 全局 PCA 截面质心分箱 → 弧长重采样 |
| **OTLC** | Mesh + **点云** | 体素降采样 sample → OT 软分配 + 向心 LC + OTC 合并 → **四级提线兜底**（见下节） |

**点云只能选 OTLC**；源为点云时 UI 自动切换，Laplacian 不可用。算法实现细节见 [`GeometryAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) **§3.5**；本节侧重 **插件侧调用链** 与 **文档/认知 vs 代码** 差异。

API：`beginTubularGrindingSession` → `runTubularGrindingStage`（须按序）；切换数据源或重置时 `clearTubularGrindingSession`。摘要写入侧栏日志 + RunInfo `[特征构建]`。

**侧栏布局**：`QTabWidget` 两页 — **中心线提取**（算法切换 + 参数同步切换）、**轨迹与投影**（模板、螺旋圈数、投影距离）。

---

### 点云中心线：调用链（插件 → 几何库）

```text
TubularGrindingDockWidget::collectParams()
  → PluginTubularGrindingParams
  → PluginPointCloudHostImpl::buildTubularGrindingGeoParams()
  → TubularGrindingParams { centerlineMethod = OtLc, inputKind = PointCloud }

runTubularGrindingStage(Centerline)
  → TubularGrindingSession（inputKind=PointCloud，跳过 mesh 构建）
  → runOtLcSkeletonCenterline(SkeletonInput{ pointXyz }, params, …, onIteration)
  → resamplePolylineToSamples + buildFrenetFrames

Host 可视化（Centerline 阶段完成后）：
  → registerTubularGrindingCenterlineLines   // 红色折线 overlay
  → registerTubularGrindingCenterlinePcaAxis // 绿色 PCA 箭头（仅调试，非提线算法）
  → buildIterationSnapshotPointsCloud × N    // OTLC 迭代 sample 点云 _迭代N
```

---

### 点云 OTLC 实际算法流程（`OtLcSkeleton.cpp`）

与论文/早期设计稿的差异见下一节「文档 vs 代码」表。

```text
1. 输入
   buildOriginalFromInput → 全量点坐标 originalPositions
   KNN(k=30) + 互 KNN 滤波 → lcAdjacency（LC 收缩用）

2. Sample 初始化（非随机下采样）
   voxelSize = bboxDiag / (N × otSampleRate)^(1/3)
   voxelDownsamplePoints → 体素质心
   每个质心 Kd-tree snap 到最近原始点 → samplePositions（仍在壳上）

3. 预处理 × otcPreSteps
   assignOriginalPointsToSamples   // 活跃 sample 根 KNN 1-NN 分簇（非完整 OT 传输矩阵）
   updateSamplePositionsFromClusters // 簇内 Sinkhorn 式迭代软权重质心
   refreshSampleMedialPositions    // 射线束 inward 融合 → 簇心内推
   otcClusterMergeStep             // 距离门控 Union-Find 合并

4. OTLC 外循环 × otLcOuterMaxIters（前 5 轮强制，之后根数达标或无合并则停）
   contractPointCloudInwardLc      // 多步约束 Laplacian + 内向法向步进
   estimatePointCloudInwardNormals // 刷新法向
   OT+合并 × otcOuterLoops
   sparseMergeSampleRoots          // 目标根数 = max(8, minRoots, bboxDiag/sectionSpacing×0.6)
   onIteration(outer+1)            // 迭代快照

5. 收尾
   sparseMergeSampleRoots（最终，×0.45 系数）
   rebuildSampleGraphEdges         // PCA 对齐边 + 排序骨干链 + 根 KNN + bridgeSampleEdgeComponents
   refineRootPositionsInward       // 各簇射线束精炼 sample 根位置

6. 四级提线（按序尝试，均过弧弦比 ≤ 4.5 门控）
   ① extractClusterOrderedPolyline(rootPositions)     → pathKind=1  「OT 分簇链」
   ② extractCenterlineFromOtSkeleton(rootAdjacency)   → pathKind=1  （须单连通且边≥根−1）
   ③ extractSliceCentroidPolyline(contractedCloud≤12k) → pathKind=0  「收缩点云截面质心」
   ④ extractOrderedCenterlinePolyline(contractedCloud)  → pathKind=2  「有序折线兜底」

7. resamplePolylineToSamples(sectionSpacingMm) → TubularCenterlineSample[]
```

**RunInfo 摘要字段**（`formatTubularGrindingStageSummaryZh`）：

| 字段 | 含义 |
|------|------|
| `centerlineOtPathKind` | 0=全局 PCA 点云截面 / 1=OT 分簇链或 sample 图 / 2=KNN 有序折线兜底 |
| `centerlineOtRootCount` | 最终活跃 sample **根**数（合并后） |
| `centerlineOtEdgeCount` | 根图无向边数（`buildRootAdjacencyFromSampleEdges` 统计） |
| `centerlineOtComponentCount` | 根图连通分量数（桥接后通常应为 1；>1 表示建边仍不足） |
| `centerlineOtKnnFallbackEdges` | **已废弃语义**：当前 OT 提线不再走 KNN 补边，该标志恒为 false |

---

### 文档 / 认知 vs 代码差异（点云中心线）

| 话题 | 文档或 UI 常见说法 | 代码实际行为 |
|------|-------------------|--------------|
| **降采样** | 早期文档写「随机下采样 otSampleRate」 | **体素滤波** + snap 最近原始点；`otSampleRate` 控制体素尺寸，非随机索引比例 |
| **OT 传输** | 「最优传输 Sinkhorn M×N」 | **最近 sample 根 1-NN 分簇** + 簇内 `exp(−‖x−y‖^2β/ε)` 加权质心迭代；无完整传输计划 |
| **提线路径「OT 分簇链」** | 易被理解为 sample 图最长路径 | **多数情况**是 `extractClusterOrderedPolyline`：合并后的 **簇心按全局 PCA 投影排序** 连成折线；**不依赖** sample 图连通 |
| **提线路径 vs 调试统计** | 「pathKind=1 且 连通分量=16」看似矛盾 | 分簇链走 ① 时 **不检查** 根图连通；摘要里的 根/边/分量 仅为调试，**不代表实际走了 sample 图最长路径** |
| **「收缩点云截面质心」** | 听起来像 OT 结果 | 实为 **LC 收缩后点云（≤12k）的全局 PCA 分箱质心**，与 Laplacian 网格路径同类兜底，**不是** OT 分簇 |
| **绿色 `_PCA轴`** | 易当成提线主轴 | 仅对 **最终折线** 做 PCA 可视化；与 OT 分簇 / 全局 PCA 兜底均无关 |
| **`centerlinePcaFallback`** | 旧注释写「false=OT sample 图」 | 现映射为 `extractPathKind != 1`；pathKind=0/2 时为 true |
| **K 邻域 UI** | 侧栏无 `pointCloudKnnK` | 代码默认 **30**（`OtLcParams.pointCloudKnnK`），未暴露 UI |
| **Laplacian K 邻域** | 侧栏控件存在 | **预留**，点云/网格 Laplacian 均不读该字段 |
| **LC 收缩** | 与网格 Laplacian 同类 | 点云 **无边塌缩/删面**；仅 KNN 图上的内向 LC + 法向步进 |
| **sample 图最长路径** | 论文主路径 | 代码为 **二级兜底**；且 `isSampleGraphUsable` 要求 **单连通 + 边≥根−1**，否则跳过 |
| **迭代快照 `_迭代N`** | 全部 sample | **活跃 sample 根**（射线束精炼后） |
| **迭代快照 `_迭代N_收缩`** | — | **LC 收缩 original 子采样**（蓝，≤8k） |

---

### 提线路径判读（调试建议）

| RunInfo 提线路径 | 实际几何来源 | 弯管/歧管预期 |
|-----------------|-------------|--------------|
| **OT 分簇链** | OT 合并根 + 射线束精炼 → PCA 排序链 | 直/弱弯管通常可用；强弯管依赖全局 PCA 排序，可能偏壳或锯齿（见弧弦比门控） |
| **收缩点云截面质心** | 收缩点云全局 PCA 分箱 | 分簇链/ sample 图均未过门控时的兜底；弯管全局 PCA 固有限制 |
| **有序折线兜底** | 收缩点云 KNN 最长路径 | 最后手段，质量最不可控 |

**采样点数**：由 `弧长 / sectionSpacingMm` 决定（通常几十～百余），**不应**出现数千点（若出现，说明折线锯齿导致弧长虚高，检查提线路径是否为 0/2）。

**调参优先级（点云）**：

1. `sectionSpacingMm` — 输出密度 + 合并/建边尺度基准  
2. `minRootsBySamples` — 防止过度合并（Auto 时 `max(15, sampleCount×5%)`）  
3. `otSampleRate` — 体素密度（越小 sample 越少、合并压力越大）  
4. `otLcOuterMaxIters` / `laplacianLambda` — 收缩与合并充分度  

---

**中心线提取页参数**（`PluginTubularGrindingParams` → `buildTubularGrindingGeoParams`）：

| 控件 | 字段 | Laplacian | OTLC（点云） |
|------|------|-----------|--------------|
| 截面间距 | `sectionSpacingMm` | PCA 分箱 + 重采样间距 | 合并/建边尺度 + 重采样间距 |
| 提取算法 | `centerlineMethod` | — | 点云强制 OtLc |
| 收缩迭代次数 | `centerlineIterations` | 网格 LC 主循环次数 | 映射为 **每轮外循环内 LC 内层步数**（`÷ otLcOuterMaxIters`） |
| 收缩强度 λ | `laplacianLambda` | 锚定权重峰值 | 同左 |
| 初始锚定强度 | `laplacianAttraction` | 前期贴原网格 | 同左 |
| K 邻域大小 | `laplacianKNeighbors` | **预留（未用）** | **预留（未用）** |
| OT 采样率 | `otSampleRate` | — | 体素降采样比例（见上） |
| OT 代价 β | `otCostBeta` | — | 簇内距离指数权重 |
| 最小根点数 | `minRootsBySamples` | — | 合并下限；0=Auto |
| OTC 预迭代 | `otcPreSteps` | — | 预处理 OT+合并轮次 |
| OTC 内层循环 | `otcOuterLoops` | — | 每轮外循环内 OT+合并次数 |
| OTLC 外层迭代 | `otLcOuterMaxIters` | — | 外循环上限（≥5 才允许早停） |

**代码默认、UI 未暴露**：`pointCloudKnnK=30`（LC 邻接 KNN）。

**轨迹与投影页**：模板类型、螺旋圈数、投影最大距离。点云投影走 Kd-tree top-K 最近邻（queryK=200，网格走射线-三角求交）。

**OTLC 迭代快照**（OSG 场景点云）：

- `_迭代N`：**活跃 sample 根**（射线束精炼后，绿/黄/红按迭代轮次）
- `_迭代N_收缩`：**LC 收缩后的 original 子采样**（蓝色，≤8k 点，观察全点云向心）
- 每轮外循环末尾 + 初始态各捕获一次；可通过点云列表显隐对比

**已知限制**（点云路径）：

- 中心线为 **单折线**，Y/T 歧管无法树形分支（Segment 对点云不可用）。  
- 未跑 Segment 时 `segments` 为空，模板阶段可能仅单管。  
- 投影为 KNN 近似，非精确射线求交。  
- 失败统一文案：`otlc centerline polyline extraction failed`（不区分建图/提线/重采样）。  

**Segment 阶段（API 保留，UI 未暴露）**：仍可通过 `runTubularGrindingStage(Segment)` 注册调试用 mesh 着色对象；点云输入 `ensureMesh` 失败。

典型调试：先看 RunInfo **提线路径** 与 **采样点数**；再看 `_迭代N` 颜色判断 sample 是否过度合并；直管验证红色 overlay 是否进腔心。

算法与参数完整表：[`GeometryAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.5。轨迹 ingress 预留：[`RobotScene/inc/TubularGrindingTrajectoryIngress.h`](../../Robot/RobotScene/inc/TubularGrindingTrajectoryIngress.h)（Phase 5，当前桩）。

## 相关文档

- SDK：[`../CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md)
- 宿主：[`../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)
- 模板 B-rep 更新：[`../../docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)
- VcgAlgorithms：[`../../Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md`](../../Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md)
