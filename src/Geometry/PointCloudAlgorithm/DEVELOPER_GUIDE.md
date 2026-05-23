# PointCloudAlgorithm 模块开发文档

## 1. 模块定位

`PointCloudAlgorithm` 是 **点云几何算法静态库**：配准、裁剪、下采样、预处理、表面重建。不依赖 Qt/OSG，不定义点云/网格领域 struct。

| 属性 | 说明 |
|------|------|
| 输出 | 静态库 `PointCloudAlgorithm.lib`（**仅**链入 `Data.dll`，x64 无独立 DLL） |
| 命名空间 | `pclalgo` |
| 依赖 | Eigen（`bin/SDK/eigen`）、CGAL 5.5.2、Boost、GMP（与 `Data` 相同） |

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
| `RegistrationRigid.h` | Eigen ICP + Kabsch |
| `RegistrationNonRigid.h` | TPS 形变 |
| `Preprocess.h` | 法线、离群、平滑、重建前管线 |
| `Reconstruction.h` | Poisson / Scale-space → triangleSoup |
| `SelfTest.h` | `runSelfTest(failures)` |

### 重建推荐流程

1. `voxelPrefilterMm > 0` 时 `downsampleVoxelGrid`
2. `preprocessForReconstruction`（离群 + PCA 法线 + MST 定向）
3. `reconstructPoisson` 或 `reconstructScaleSpace`
4. 一键：`reconstructPoissonAuto`

闭合良好、法线可靠 → **Poisson**；噪声大 → 先试 **Scale-space** 或加大体素预滤波。

---

## 4. Data 薄包装

[`PointCloudBackendOps.h`](../Data/inc/PointCloudBackendOps.h)（`point_cloud_backend_ops`）：

- `downsamplePointCloud`
- `applyRigidTransformToPointCloud`
- `reconstructMeshFromPointCloudPoisson`

耗时调用宜在 Widget `JobSystem` 后台线程执行，UI 线程仅 `setPointBuffers` / `setTriangleSoup`。

---

## 5. 自检

```cpp
std::vector<std::string> failures;
const bool ok = pclalgo::runSelfTest(failures);
```

---

## 6. 相关文档

- 后端数据：[`../Data/DEVELOPER_GUIDE.md`](../Data/DEVELOPER_GUIDE.md)
- 刚体矩阵：[`../GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md)
