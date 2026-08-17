# PointCloudAlgorithm 模块开发文档

## 1. 模块定位

`PointCloudAlgorithm` 是 **点云几何算法静态库**：配准、裁剪、下采样、预处理、表面重建。不依赖 Qt/OSG，不定义点云/网格领域 struct。

| 属性 | 说明 |
|------|------|
| 输出 | 静态库 `PointCloudAlgorithm.lib`（**仅**链入 `Data.dll`，x64 无独立 DLL） |
| 命名空间 | `pclalgo` |
| 依赖 | Eigen（`bin/SDK/eigen`）、CGAL 5.5.2、Boost、GMP、TBB（可选，用于并行化） |

---

## 2. 数据契约

**Breaking v2**：ICP/RANSAC 输入应为**同一坐标系**下两组 `xyz`（推荐世界系，由 Data/Host 经 `worldMatrix` 变换后传入）。本库不感知 `worldMatrix`。

与 [`PointCloudBackendData`](../../Data/Data/inc/PointCloudBackendData.h) 对齐：

| 缓冲 | 布局 |
|------|------|
| 点云 xyz | `3*N` float，单位 **mm** |
| 顶点 rgba | 可选 `4*N`，0..1 |
| 法线 | `3*N`，与 xyz 同序 |
| 网格 soup | `9*T` float，每三角 3 顶点 xyz（同 [`MeshBackendData`](../../Data/Data/inc/MeshBackendData.h)） |

矩阵一律 **Eigen**；刚体变换为列向量语义 `p' = T * p`（与 `GeometryEngine::composeColumn` 一致）。

---

## 3. API 总览

| 头文件 | 功能 |
|--------|------|
| `PointCloudBuffer.h` | 点数、紧凑拷贝 |
| `Measure.h` | 包围盒、质心、平均间距 |
| `Transform.h` | 刚体变换点坐标 |
| `Crop.h` | AABB / 球裁剪 |
| `Downsample.h` | CGAL 体素 / 随机下采样 |
| `RegistrationRigid.h` | 点-点 / 点-面 ICP + 法线门控 |
| `RegistrationGlobal.h` | FPFH + 特征匹配 + RANSAC + Kabsch（`rigidRegisterFeatureRansac`） |
| `RegistrationNonRigid.h` | TPS 形变 |
| `RegistrationSpare.h` | **SPARE** 非刚性配准（对称点-面 + 变形图 + ARAP；点云/网格 soup） |
| `RegistrationSdf.h` | **SDF/DDF** 混合非刚性配准（粗场残差 + 细默认点-面；独立于 SPARE） |
| `Preprocess.h` | 法线、离群、平滑、重建前管线 |
| `ReconstructionPoisson.h` | Poisson 隐式重建（定向点云）；`reconstructPoisson` / `Auto` |
| `ReconstructionScaleSpace.h` | Scale-space 重建（仅坐标）；`reconstructScaleSpace` |
| `Reconstruction.h` | 聚合转发（兼容旧 `#include`） |
| `ReconstructionConfig.h` | 重建配置（质量级别、自动下采样、并行化控制） |
| `ParallelUtils.h` | 并行化工具类（TBB检测、线程数管理） |
| `SelfTest.h` | `runSelfTest(failures)` |

### 重建推荐流程（速查）

| 路径 | 入口 API | 适用场景 |
|------|----------|----------|
| Poisson 一键 | `reconstructPoissonAuto` / `reconstructPoissonAutoWithConfig` | 闭合曲面、可估法线；插件默认重建 |
| Poisson 手动 | `reconstructPoisson`（自带法线） | 已预处理、法线质量可控 |
| Scale-space | `reconstructScaleSpace` / `reconstructScaleSpaceWithConfig` | 噪声大、无法线或法线不可靠 |

闭合良好、法线可靠 → **Poisson**；噪声大、缺法线 → **Scale-space** 或 Poisson 前加大体素预滤波。

---

### 3.1 Poisson 与 Scale-space：流程与区别

二者均基于 CGAL，输出统一为 **triangle soup**（`9*T` float，每三角 3 顶点 xyz，单位 mm）。实现见 [`ReconstructionPoisson.cpp`](source/ReconstructionPoisson.cpp) / [`ReconstructionScaleSpace.cpp`](source/ReconstructionScaleSpace.cpp)；旧 [`Reconstruction.h`](inc/Reconstruction.h) 为聚合转发。

#### 算法本质

| 维度 | Poisson（隐式重建） | Scale-space（尺度空间重建） |
|------|---------------------|-----------------------------|
| CGAL 入口 | `CGAL::poisson_surface_reconstruction_delaunay` | `CGAL::Scale_space_surface_reconstruction_3` |
| 输入 | **点坐标 + 定向法线**（`Point_with_normal`） | **仅点坐标**（`Point_3`） |
| 原理 | 将定向点云视为泊松方程约束，求解隐式指示函数后提取等值面 | 多尺度平滑点集，在尺度序列上提取稳定表面 |
| 对法线要求 | **强依赖**；法线方向错误会导致翻面或空洞 | **不依赖法线**；内部自行平滑与连面 |
| 典型优势 | 闭合、水密倾向好；细节与平滑度可通过 spacing / 平滑参数调节 | 对噪声、非均匀采样更宽容；无需 MST 定向 |
| 典型劣势 | 法线差或开口扫描易失败；需预处理链 | 尖锐特征易钝化；`smoothIterations` 过大可能过度平滑 |

#### Poisson 流程

```
xyz [+ 可选已有 normals]
    │
    ├─ reconstructPoissonAuto / AutoWithConfig
    │       └─ preprocessForReconstruction
    │              1. voxelPrefilterMm > 0 → downsampleVoxelGrid
    │              2. outlierRemovalPercent > 0 → removeOutliers (k=24)
    │              3. estimateNormalsPca (k=12)
    │              4. orientNormalsMst (k=12)
    │
    ├─ reconstructPoissonWithConfig
    │       └─ 超 maxPointsForReconstruction 时自动体素下采样 + 重算法线
    │       └─ reconstructPoisson（调用方已提供 normals）
    │
    └─ reconstructPoisson（底层）
            spacingMm ≤ 0 → computeAverageSpacingMm(xyz, 6)
            poisson_surface_reconstruction_delaunay → Surface_mesh → triangleSoup
```

**默认 Poisson 参数**（`reconstructPoissonAuto` / `WithConfig`）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `spacingMm` | 0（自动平均间距） | 八叉树深度 / 体素尺度 |
| `smAngleDeg` | 20° | 表面平滑：角度阈值 |
| `smRadiusRel` | 30 | 表面平滑：半径（相对 spacing） |
| `smDistanceRel` | 0.375 | 表面平滑：距离（相对 spacing） |

Data 层：`reconstructMeshFromPointCloudPoisson` → `reconstructPoissonAuto`；若点云**无法线**且走显式 Poisson 路径会报错 `"Poisson requires normals"`。

#### Scale-space 流程

```
xyz
    │
    ├─ reconstructScaleSpaceWithConfig
    │       └─ 超 maxPointsForReconstruction 时自动体素下采样（无法线重算）
    │
    └─ reconstructScaleSpace（底层）
            构造 Scale_space_surface_reconstruction_3
            increase_scale(smoothIterations)   // 尺度提升 / 平滑迭代
            reconstruct_surface()              // 在当前尺度提取三角面
            facets → triangleSoup（直接写顶点，不经 Surface_mesh）
            orientTriangleSoupWinding（焊点 + repair/orient_polygon_soup + 封闭体体积整体翻转）
```

**默认 Scale-space 参数**：

| 参数 | 默认值（Balanced） | 含义 |
|------|-------------------|------|
| `smoothIterations` | Fast=2 / Balanced=4 / Quality=6 | `increase_scale` 迭代次数；越大曲面越平滑、细节越少 |
| `meshingRadiusMm` | 0（自动：包围盒对角线 × 0.05） | 当前实现中仅计算默认值，**未传入 CGAL**（见 `ReconstructionScaleSpace.cpp` 中 `(void)meshingRadiusMm`） |

Scale-space **不调用** `preprocessForReconstruction`；离群剔除、法线估计、MST 定向均不在此路径内。需要时可由调用方在重建前自行 `downsampleVoxelGrid` / `removeOutliers`。

#### 配置 API 对比

| API | 预处理 | 超点数处理 | 质量档位影响 |
|-----|--------|------------|--------------|
| `reconstructPoissonAutoWithConfig` | 完整 `preprocessForReconstruction` | 体素下采样 | 体素、离群%、（间接影响法线） |
| `reconstructPoissonWithConfig` | 无（调用方提供 normals） | 体素下采样 + `estimateNormalsPca` | 仅下采样阈值 |
| `reconstructScaleSpaceWithConfig` | 无 | 体素下采样 | `smoothIterations` |

```cpp
// Poisson 一键（推荐默认）
pclalgo::ReconstructionConfig config;
config.quality = pclalgo::ReconstructionQuality::Balanced;
pclalgo::reconstructPoissonAutoWithConfig(xyz, soup, config, &err);

// Scale-space（噪声 / 无法线）
pclalgo::reconstructScaleSpaceWithConfig(xyz, soup, config, &err);
// 或底层：reconstructScaleSpace(xyz, soup, /*smoothIterations=*/4, /*meshingRadiusMm=*/0.0, &err);
```

#### 选型建议

1. **扫描闭合、密度均匀**：`reconstructPoissonAutoWithConfig`，`Quality` 档 + 适当 `voxelPrefilterMm`。
2. **开口工件、法线难定向**：先试 `reconstructScaleSpaceWithConfig`，`smoothIterations` 从 4 起调。
3. **点云已带可靠法线**（如激光标定输出）：可直接 `reconstructPoisson`，跳过 Auto 预处理以保留细节。
4. **重建后网格质量**：本模块只负责 soup；简化 / 平滑 / 修复见 [`VcgAlgorithms`](../VcgAlgorithms/DEVELOPER_GUIDE.md) 与 `reconstructAndPostProcess`（当前后处理管线绑定 **Poisson**，非 Scale-space）。

---

### 3.2 配置版本 API（推荐）

使用 `ReconstructionConfig` 进行更精细的控制：

```cpp
pclalgo::ReconstructionConfig config;
config.quality = pclalgo::ReconstructionQuality::Balanced;
config.maxPointsForReconstruction = 500000;  // 自动下采样阈值
config.enableParallel = true;                 // 启用 TBB 并行化

pclalgo::reconstructPoissonAutoWithConfig(xyz, soup, config, &err);
```

质量级别影响参数：

| 级别 | 体素预滤波 | 平滑迭代 | 离群移除 |
|------|-----------|----------|----------|
| Fast | 2.0 mm | 2 | 3% |
| Balanced | 1.0 mm | 4 | 5% |
| Quality | 0.5 mm | 6 | 7% |

### 3.3 并行化

当 `CGAL_LINKED_WITH_TBB` 定义时（需链接 TBB），以下算法自动使用 `CGAL::Parallel_tag`：

- `pca_estimate_normals` / `jet_estimate_normals`
- `remove_outliers` / `bilateral_smooth_point_set`
- `compute_average_spacing`

通过 `ParallelUtils::isParallelEnabled()` 可运行时控制。

### 3.4 全局粗配准（`RegistrationGlobal.h`）

用于 `geometry_backend_ops::alignScanToTemplateRegistration` 世界系粗配（`enableRansacCoarseMatch=true`）。

| 项 | 说明 |
|----|------|
| `rigidRegisterFeatureRansac` | 体素下采样 → SPFH/FPFH → 互匹配+ratio test → RANSAC → Kabsch → 可选点-面 ICP（`refineWithIcp`） |
| `RigidRegisterRansacParams` | `featureVoxelMm`、`inlierDistanceMm`、`minInliers`、`maxIterations` 等；0 表示按 modelDiag 自动 |
| 失败 | 不阻断流水线，继续粗 ICP |

预对齐插件路径**跳过** RANSAC；见 [`docs/template_brep_pointcloud_update.md`](../../../docs/_archive/template_brep_pointcloud_update.md)。

### 3.5 SPARE 非刚性配准（`RegistrationSpare.h`）

移植自 [SPARE: Symmetrized Point-to-Plane Distance](https://arxiv.org/abs/2405.20188) 核心求解器（研究用途；源码专利声明见 `bin/SDK/spare-main-extracted/spare-main/README.md`）。基础设施复用本库 CGAL/`KdTreePointSet`/ICP/下采样；**不依赖** OpenMesh 或 `GeometryAlgorithm.dll`。

**原理通俗说明**（对称点-面、粗/细阶段、Welsch、ARAP、调参与流水线）：见 [`docs/spare_nonrigid_registration.md`](../../../docs/_archive/spare_nonrigid_registration.md)。

| 入口 | 说明 |
|------|------|
| `spareRegisterPointClouds` | 点云 → 点云；`xyz` + 法线均为 `3*N` float（mm） |
| `spareRegisterMeshSoupToTarget` | 网格 soup（`9*T`）→ 点云目标 |
| `spareRegisterMeshSoupToMeshSoup` | 网格 → 网格 |
| `SpareRegisterParams` | 采样半径比、平滑/旋转/ARAP 权重、粗/细阶段、`rigidPreAlign`（`rigidRegisterPointToPlaneIcp`）、`voxelPrefilterMm` 等 |

内部实现位于 `source/spare/`：`SpareSurfaceBuild`（CGAL soup→`Surface_mesh`）、`SpareNodeSampler`（点云 FPS / 网格边图 Dijkstra 测地）、`SpareSolver`（对称点-面 + Welsch + 变形图）。

**自检**：`runSelfTest` 含 `spare.ok`（合成平面点云，5 轮外迭代）。与参考实现的数值对比可用手动回归：将 `bin/SDK/spare-main-extracted/spare-main/data/test{1,2,3}` 的 PLY 导入后调用 `spareRegisterPointClouds`（test3）或 mesh 入口（test1/2），对比 `meanErrorMm` 与 `our_params.txt` 中 `init_gt_mean_errs` 同量级。

### 3.6 SDF/DDF 混合非刚性配准（`RegistrationSdf.h`）

自研模块（**不修改** `spare/` / `RegistrationSpare`）。粗阶段用目标表面 **DDF 有向距离**（或可选有符号 SDF）作数据项，细阶段默认 **点-面**；目标场可体素缓存。原理与调参见 [`docs/_archive/sdf_nonrigid_registration.md`](../../../docs/_archive/sdf_nonrigid_registration.md)。

| 入口 | 说明 |
|------|------|
| `sdfRegisterPointClouds` | 点云 → 点云 |
| `sdfRegisterMeshSoupToTarget` | 网格 soup → 点云目标 |
| `sdfRegisterMeshSoupToMeshSoup` | 网格 → 网格 |
| `SdfRegisterParams` | `fieldMode` / `fieldVoxelMm` / `fineDataTerm`、变形图与 ARAP 权重、粗/细开关等 |

内部：`source/sdf/`（`DistanceField`、`SdfNodeSampler`、`SdfDeformSolver`）。自检：`sdf.ok`。

插件：点云侧栏配准方法下拉「SDF/DDF 非刚性」→ Host `nonRigidRegisterSdf`（1.17.0+）。

---

## 4. Data 薄包装

[`PointCloudBackendOps.h`](../../Data/Data/inc/PointCloudBackendOps.h)（`point_cloud_backend_ops`）覆盖全部 `pclalgo` API：下采样、裁剪、度量、变换、离群/平滑、法线、预处理、ICP、TPS、**SPARE**、**SDF/DDF**、Poisson/Scale-space 重建。

常用入口：

- `downsamplePointCloudVoxel` / `downsamplePointCloudRandom`
- `applyRigidTransformToPointCloud`
- `rigidRegisterPointCloudsIcp`
- `nonRigidRegisterPointCloudsSpare` / `nonRigidRegisterPointCloudToMeshSpare` / `nonRigidRegisterMeshSpare`（`PointCloudSpareParams`）
- `nonRigidRegisterPointCloudsSdf` / `nonRigidRegisterPointCloudToMeshSdf` / `nonRigidRegisterMeshSdf`（`PointCloudSdfParams`）
- `reconstructMeshFromPointCloudPoisson`（Poisson Auto）

插件侧映射见 [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../../Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) §点云 SDK。

---

## 5. 自检

```cpp
std::vector<std::string> failures;
const bool ok = pclalgo::runSelfTest(failures);
```

检查并行化状态：

```cpp
const bool tbbAvailable = pclalgo::ParallelUtils::isTbbAvailable();
const int threads = pclalgo::ParallelUtils::getThreadCount();
```

---

## 6. vcglib 网格后处理集成

`PointCloudAlgorithm` 负责点云→mesh 重建（CGAL Poisson / Scale-space）。重建后的网格后处理（简化、平滑、修复、重网格）由独立模块 [`VcgAlgorithms`](../VcgAlgorithms/DEVELOPER_GUIDE.md) 提供，基于 [vcglib](https://github.com/cnr-isti-vclab/vcglib)。

| 能力 | 模块 | 说明 |
|------|------|------|
| Poisson / Scale-space 重建 | `PointCloudAlgorithm`（本模块） | CGAL 实现 |
| 网格简化 | `VcgAlgorithms` | quadric-edge-collapse |
| 网格平滑 | `VcgAlgorithms` | Laplacian / Implicit Fairing |
| 网格修复 | `VcgAlgorithms` | 去退化/重复/非流形/填孔 |
| 各向同性重网格 | `VcgAlgorithms` | 均匀三角形分布 |
| 重建+后处理管线 | `VcgAlgorithms` | `reconstructAndPostProcess` 调用本模块 Poisson + vcglib 后处理 |

**曲面重构（网格 → NURBS B-rep）** 不在本模块，而在 [`GeometryAlgorithm/MeshSurfaceReconstruction`](../GeometryAlgorithm/inc/MeshSurfaceReconstruction.h)（AMRTO 式调和 UV 栅格 + NURBS 最小二乘拟合）。插件侧栏「曲面重构」经 Data → `geoalgo::reconstructBrepFromMeshSoup` 调用。详见 [`docs/mesh_surface_reconstruction.md`](../../../docs/_archive/mesh_surface_reconstruction.md)。

`Data.dll` 的 `PointCloudBackendOps` 暴露统一 soup-based 接口，运行时 `LoadLibrary("VcgAlgorithms.dll")` 调用。

---

## 7. 相关文档

- 后端数据：[`../Data/DEVELOPER_GUIDE.md`](../../Data/Data/DEVELOPER_GUIDE.md)
- 刚体矩阵：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md)
- vcglib 网格后处理：[`../VcgAlgorithms/DEVELOPER_GUIDE.md`](../VcgAlgorithms/DEVELOPER_GUIDE.md)
- 模板 B-rep 配准：[`../../docs/template_brep_pointcloud_update.md`](../../../docs/_archive/template_brep_pointcloud_update.md)
- 性能优化方案：[`../../docs/mesh_reconstruction_optimization/`](../../../docs/_archive/mesh_reconstruction_optimization/)
- vcglib 集成方案：[`../../docs/vcglib_integration/`](../../../docs/_archive/vcglib_integration/)
