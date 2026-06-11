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

与 [`PointCloudBackendData`](../Data/inc/PointCloudBackendData.h) 对齐：

| 缓冲 | 布局 |
|------|------|
| 点云 xyz | `3*N` float，单位 **mm** |
| 顶点 rgba | 可选 `4*N`，0..1 |
| 法线 | `3*N`，与 xyz 同序 |
| 网格 soup | `9*T` float，每三角 3 顶点 xyz（同 [`MeshBackendData`](../Data/inc/MeshBackendData.h)） |

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
| `Preprocess.h` | 法线、离群、平滑、重建前管线 |
| `Reconstruction.h` | Poisson / Scale-space → triangleSoup |
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

二者均基于 CGAL，输出统一为 **triangle soup**（`9*T` float，每三角 3 顶点 xyz，单位 mm）。实现见 [`Reconstruction.cpp`](source/Reconstruction.cpp)。

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
| `meshingRadiusMm` | 0（自动：包围盒对角线 × 0.05） | 当前实现中仅计算默认值，**未传入 CGAL**（见 `Reconstruction.cpp` 中 `(void)meshingRadiusMm`） |

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

用于 `geometry_backend_ops::alignScanToTemplateRegistration` 的非预对齐路径（`enableRansacCoarseMatch=true` 且 `scanAlreadyInTemplateFrame=false`）。

| 项 | 说明 |
|----|------|
| `rigidRegisterFeatureRansac` | 体素下采样 → SPFH/FPFH → 互匹配+ratio test → RANSAC → Kabsch → 可选点-面 ICP（`refineWithIcp`） |
| `RigidRegisterRansacParams` | `featureVoxelMm`、`inlierDistanceMm`、`minInliers`、`maxIterations` 等；0 表示按 modelDiag 自动 |
| 失败 | 不阻断流水线，继续粗 ICP |

预对齐插件路径**跳过** RANSAC；见 [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)。

---

## 4. Data 薄包装

[`PointCloudBackendOps.h`](../Data/inc/PointCloudBackendOps.h)（`point_cloud_backend_ops`）覆盖全部 `pclalgo` API：下采样、裁剪、度量、变换、离群/平滑、法线、预处理、ICP、TPS、Poisson/Scale-space 重建。

常用入口：

- `downsamplePointCloudVoxel` / `downsamplePointCloudRandom`
- `applyRigidTransformToPointCloud`
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

`Data.dll` 的 `PointCloudBackendOps` 暴露统一 soup-based 接口，运行时 `LoadLibrary("VcgAlgorithms.dll")` 调用。

---

## 7. 相关文档

- 后端数据：[`../Data/DEVELOPER_GUIDE.md`](../Data/DEVELOPER_GUIDE.md)
- 刚体矩阵：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md)
- vcglib 网格后处理：[`../VcgAlgorithms/DEVELOPER_GUIDE.md`](../VcgAlgorithms/DEVELOPER_GUIDE.md)
- 模板 B-rep 配准：[`../../docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)
- 性能优化方案：[`../../docs/mesh_reconstruction_optimization/`](../../docs/mesh_reconstruction_optimization/)
- vcglib 集成方案：[`../../docs/vcglib_integration/`](../../docs/vcglib_integration/)
