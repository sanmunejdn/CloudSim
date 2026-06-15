# 网格曲面重构（BrepModel）

将三角网格重构为分片 B 样条 B-rep，输出新的 `BrepModel` 后端，**不覆盖**源网格。

## 管线

```
三角 soup (mm)
  → [阶段 1] Vcg 修复 + 法矢光顺 (Ch2, Data 层 preprocess)
  → [阶段 2] 四边域分块 (Ch3.2 partition)
  → [阶段 3] PCA 物理栅格采样 (Ch3.2 sample)
  → [阶段 4] NURBS 曲面拟合 / 平面回退 (NurbsSurfaceFitting + GeomAPI 回退)
  → [阶段 5] 边界 C² 混合 (Ch3.3 BoundaryBlend)
  → [阶段 6] 交汇 C² 混合 (Ch3.3 JunctionBlend)
  → [阶段 7] B 样条局部光顺 (Ch4)
  → [阶段 8] Compound 装配 + 输出校验 → ShapeHandle → BrepBackendData
```

**1.13.0+** 支持按阶段单步执行（`MeshSurfaceReconstructSession`）；**1.12.0** 仍可通过 `reconstructBrepFromMeshSoup` / `reconstructSurfaceFromMesh` 一次跑完全流程。

## 分阶段 API（1.13.0+）

| 层 | 符号 |
|----|------|
| UI | `PointCloudDockWidget::runSurfaceReconStage` |
| SDK | `beginMeshSurfaceReconstructSession` / `runMeshSurfaceReconstructStage` / `clearMeshSurfaceReconstructSession` |
| Host | `PluginPointCloudHostImpl` 会话 map（`sr_<n>`），阶段顺序校验 `isNextPluginStage` |
| Data | `preprocessMeshSoupForSurfaceReconstruct`、`createMeshSurfaceReconstructSession`、`runMeshSurfaceReconstructStage` |
| Geo | `geoalgo::MeshSurfaceReconstructSession`、`runMeshSurfaceReconstructStage` |

预处理阶段可选 `exportPreprocessedMeshToScene`：注册临时 `Model`（`源显示名_预处理后`），重置/切换网格时 `removeBackendObject` 清理。

**分块**阶段完成后自动注册 `源显示名_分块着色` 网格：每片三角使用不同顶点色（HSV 黄金角分布），便于目视检查分块边界。

阶段完成时 `PluginMeshSurfaceReconstructReport::stageSummaryZh` 提供中文摘要（侧栏日志 + RunInfo `[曲面重构]`）。

## 装配与校验

| 步骤 | 实现 | 说明 |
|------|------|------|
| 装配 | `MeshSurfaceReconstructionAssemble.cpp` | 开放曲面用 `TopoDS_Compound`，不做 Sewing |
| 三角化门禁 | `MeshSurfaceReconstructionValidate.cpp` | 单面三角数 ≤ 8000，禁止空 mesh |
| 包围盒门禁 | 同上 | 输出对角线 / 输入对角线 ≤ 3.0 |

拟合失败或极点/face 校验未过的分片自动回退为平面四边形。

## 可调参数一览

| 参数 | 默认值 | 所属阶段 | 说明 |
|------|--------|----------|------|
| `normalSmoothIterations` | 6 | 预处理 | 法矢光顺迭代次数 |
| `featureThresholdC0` | 0.8 | 预处理 | 特征阈值 c0 |
| `runVcgRepairFirst` | true | 预处理 | 分块前是否做 Vcg 修复 |
| `runIsotropicRemesh` | true | 预处理 | 修复后、光顺前各向同性重网格 |
| `remeshTargetEdgeLengthMm` | 0 | 预处理 | 目标边长 mm；0=修复后边长中位数 |
| `remeshIterations` | 3 | 预处理 | VCG 重网格迭代次数 |
| `remeshFeatureAngleDeg` | 0 | 预处理 | 特征边保护角（度）；0=由 `featureThresholdC0` 推导 |
| `patchCountHint` | 0（自动） | 分块 | 目标分块数，0=自动 `sqrt(面数/80)` |
| `partitionNormalSmoothIters` | 2 | 分块 | 分块前法向平滑迭代；UI 可调，0=保留锐角 |
| `featureAnglePercentile` | 0.88 | 分块 | 特征棱角度百分位；UI 可调，越低切分越多 |
| `samplesPerPatchEdge` | 16 | 栅格采样 | 每边采样点数（`targetUvSpacingMm=0` 时生效） |
| `targetUvSpacingMm` | 0 | 栅格采样 | UV 目标间距 mm；>0 时按物理间距自适应每边格数 |
| `minSamplesPerEdge` | 4 | 栅格采样 | 自适应间距时每边最少格数 |
| `maxSamplesPerEdge` | 0 | 栅格采样 | 自适应间距时每边最多格数；0=无上限（单 patch 硬顶 4096 点） |
| `maxFitGridPerEdge` | 9 | NURBS 拟合 | 拟合前每边最多格数；0=不降采样 |
| `fitUvSpacingMm` | 0 | NURBS 拟合 | 拟合 UV 间距 mm；0=仅用 `maxFitGridPerEdge` |
| `sampleRateFactor` | 2.0 | 栅格采样 | AMRTO k_sample，自适应 d_u/d_v |
| `controlPointDensityFactor` | 0.5 | NURBS 拟合 | AMRTO k_type_gemodl，控制点密度 |
| `fitMode` | ApproxFixedCtrlpts | NURBS 拟合 | 1 插值 / 2 最小二乘+控制点 / 3 Centripetal / 4 Centripetal+控制点 |
| `parameterGridMode` | 1 | 栅格采样 | 1 全域 / 2 内缩 / 3 仅内部 |
| `blendStripWidth` | 0（自动） | 边界/交汇混合 | C² 混合带宽度 |
| `fairingEpsilon` | 1e-3 | 光顺 | 光顺收敛阈值 |
| `fairingMaxIterations` | 50 | 光顺 | 光顺最大迭代数 |
| `tessellateLinearDeflectionMm` | 0.1 | 装配 | B-rep 三角化线性偏差精度 |

## 手工验收

### 全流程（1.12+）

1. 点云 → Poisson Auto 重建，得到 `Model` 网格
2. 侧栏「网格后处理」区选中该网格
3. 「曲面重构」区默认参数 → **全流程**
4. 树中出现新 `BrepModel`，自动选中；原网格 id 与面数不变
5. 视口可拾取 B-rep 面/边；导出 STEP 可在外部 CAD 打开
6. RunInfo 输出 patch 数、max deviation、光顺指标；失败时有明确错误（如包围盒无效、三角化过密）

### 分阶段调试（1.13+）

1. 同上选中网格，勾选「预处理后写入场景网格」
2. 依次点 **预处理** → **分块** → … → **装配输出**（不可跳步）
3. 每步侧栏日志与 RunInfo 出现中文摘要（三角数、分块数、采样点数等）
4. 预处理后树中出现 `*_预处理后` 网格；装配后新 `BrepModel` 与全流程一致
5. **重置会话** 或切换网格对象后须从预处理重新开始

## API 链路（全流程）

| 层 | 符号 |
|----|------|
| UI | `PointCloudDockWidget` → `IPluginPointCloudHost::reconstructSurfaceFromMesh` |
| Host | `runMeshToBrepJob` → `registerAdoptedBrepAndLoadScene(skipRebase=false)` → `inheritBrepVisualPoseFromSourceMesh`（内层质心与源网格对齐） |
| Data | `geometry_backend_ops::reconstructBrepFromMeshSoup` |
| Geo | `geoalgo::reconstructBrepFromMeshSoup` |
| Vcg | `vcgalgo::smoothMeshByNormalAdjustment` |

## 自检

```cpp
std::string err;
const bool ok = geoalgo::runSelfTest(&err);  // 含 meshSurfaceReconstruct
```

## 分块算法（v3）

分块将三角面聚类为若干 `QuadPatch`，供后续采样/拟合。v3 针对高曲率区域 confetti 碎片问题，借鉴 AMRTO/GMCG「大片连续 chart」原则，在三角 mesh 上内置实现。

### 邻接建图 O(F)

用 `edge→face-pair` 哈希表构建：
- `fullAdj`：所有共边面，用于 patch 邻接与 orphan 分配
- `smoothAdj`：仅二面角低于特征阈值的共边面，用于测地 Voronoi

### 分区法向平滑 + 百分位特征棱

分块前对法向做 `partitionNormalSmoothIters` 次邻接平均（默认 2），仅用于特征判定，不改几何。

特征阈值：`max(featureThresholdC0, P88)`，再 clamp 到 `[0.5×median, 3×median]`。光滑曲面上的伪特征边不再切断 `smoothAdj`。

### FPS 种子 + 测地多源 Voronoi

1. FPS 在面质心空间选 `patchCountHint` 或 `sqrt(面数/80)` 个种子
2. 在 `smoothAdj` 上多源 Dijkstra（边权=质心距离），每面归属测地最近种子
3. 未连通区域经 `fullAdj` 扩散归入邻片

替代 v2 的有上限 BFS + orphan 首邻接，消除高曲率 confetti。

### 超大片分裂 / 连通分量清理 / 合并 v2

- 片大小 > `1.25×maxFacesPerPatch` 时子图内 FPS 二分
- 多连通小分量按法向相似度并入邻片
- 合并：`minFaces = max(100, F/(2×targetPatches))`；评分 = 0.5×平面性 + 0.2×纵横比 + 0.3×法向相似；第二遍强制合并 < `targetAvg/4` 的片

### 分块报告字段

| 字段 | 含义 |
|------|------|
| `minFacesPerPatch` / `maxFacesPerPatch` | 合并后各片三角数范围 |
| `smallPatchCount` | 合并前小于阈值的碎片片数 |
| `avgFacesPerPatch` | 平均每片三角数 |

## 栅格采样（v3）

实现：`MeshSurfaceReconstruction/PatchParameterize.cpp`。

### 策略概览

每 patch 在 PCA 切平面上生成物理等距 UV 栅格，再投影到曲面取 3D 采样点，写入 `QuadPatch::sampleXyz`。

```
PCA 框（顶点 + 面心）→ 物理等距栅格 → PatchClosestAccel 曲面投影  [path: pca]
        ↓ 调和 UV 可用
AMRTO 参数域栅格 + 重心回投 3D（优先）                          [path: amrto-harmonic]
        ↓ 质量未过
调和 UV 面心栅格（优于 PCA 时采用）                              [path: harmonic]
        ↓ 质量仍未过
面心锚定回退（UV 格点 → 最近面心 3D）                           [path: pca-centroid]
```

### AMRTO 式 NURBS 拟合

实现：`MeshSurfaceReconstruction/NurbsSurfaceFitting.cpp` + `InitialBsplinePatch.cpp`。

- 采样：`computeAmrtoGridResolution` 按 UV 跨度计算 `d_u/d_v` 与控制点数
- 拟合：两阶段 Eigen 最小二乘 B 样条（权重=1）；失败时回退 `GeomAPI_PointsToBSplineSurface`（Centripetal）
- 输出：`Handle(Geom_BSplineSurface)` → `BRepBuilderAPI_MakeFace`

### 质量门禁 `passesSampleQuality`

| 指标 | 阈值 | 含义 |
|------|------|------|
| `sampleDiag / patchDiag` | ≥ 0.75 | 采样点包围盒应覆盖 patch 主体 |
| `uniqueSampleRatio` | ≥ 0.65 | 0.1mm 量化后唯一点占比，抑制投影塌缩 |

未通过门禁时依次尝试调和 UV（仅小 patch）、面心锚定回退。

### 面心锚定回退

高曲率或大 patch 上，PCA 栅格经最近三角投影可能将多点塌缩到同一曲面片（典型：patch 12 覆盖率从 30% 恢复至 98%）。回退路径在 UV 平面保持等距栅格，每个格点映射到 PCA 坐标下**最近三角面心**的 3D 位置，保证采样贴曲面且空间分散。

### 性能约束

| 项 | 值 |
|----|-----|
| 单 patch 栅格上限 | 4096 点 |
| `maxSamplesPerEdge=0` 时每边 cap | 48 |
| 调和 UV 最大面数 | 3000 |
| 调和迭代轮数 | 80 |
| 大 patch 调和 | 仅 `charLen ≤ 300mm` 且质量优于 PCA 时采用 |

### 采样点云导出

`geoalgo::buildSamplePointsCloud`（`MeshSurfaceReconstruction.cpp`）将各 patch `sampleXyz` 合并为场景点云，供目视检查覆盖情况。Host 层注册 `samplePointsBackendId`。

### B 样条拟合拒因统计

`InitialBsplinePatch.cpp` 按 patch 记录 `fitRejectReason`，`MeshSurfaceReconstruction.cpp` 聚合为报告字段：

| 字段 | 含义 |
|------|------|
| `fitRejectApprox` | `GeomAPI` 逼近失败 |
| `fitRejectPole` | 极点移动/校验失败 |
| `fitRejectFitGrid` | 拟合栅格不足 |
| `fitRejectFullGrid` | 全栅格校验失败 |
| `fitRejectMakeFace` | `BRepBuilderAPI_MakeFace` 失败 |

拟合阶段摘要见 RunInfo `[曲面重构]` 与侧栏日志（含拒因 breakdown）。

## 已知限制

- C² 混合与光顺为简化实现，复杂模型可能以平面片为主
- 特征棱阈值自适应基于角度中位数，极端分布模型可能需要手动调 `featureThresholdC0`
- 面心锚定回退在极高曲率 patch 上 `uniqueSampleRatio` 可能略低于 0.65，但空间覆盖通常已足够（如 patch 6）

## 相关文档

- [`GeometryAlgorithm/DEVELOPER_GUIDE.md`](../src/Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.4
- [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../src/Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md) §曲面重构
- [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) §点云 SDK
