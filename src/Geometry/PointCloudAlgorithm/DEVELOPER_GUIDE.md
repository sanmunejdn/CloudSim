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

### 重建推荐流程

1. `voxelPrefilterMm > 0` 时 `downsampleVoxelGrid`
2. `preprocessForReconstruction`（离群 + PCA 法线 + MST 定向）
3. `reconstructPoisson` 或 `reconstructScaleSpace`
4. 一键：`reconstructPoissonAuto`

闭合良好、法线可靠 → **Poisson**；噪声大 → 先试 **Scale-space** 或加大体素预滤波。

### 3.1 配置版本 API（推荐）

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

### 3.2 并行化

当 `CGAL_LINKED_WITH_TBB` 定义时（需链接 TBB），以下算法自动使用 `CGAL::Parallel_tag`：

- `pca_estimate_normals` / `jet_estimate_normals`
- `remove_outliers` / `bilateral_smooth_point_set`
- `compute_average_spacing`

通过 `ParallelUtils::isParallelEnabled()` 可运行时控制。

### 3.1 全局粗配准（`RegistrationGlobal.h`）

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
