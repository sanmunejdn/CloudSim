# VcgAlgorithms 模块开发文档

## 1. 模块定位

`VcgAlgorithms` 是基于 vcglib 的 **网格后处理独立 DLL**：简化、平滑、修复、各向同性重网格，以及 CGAL 重建 + vcglib 后处理管线。

| 属性 | 说明 |
|------|------|
| x64 输出 | `VcgAlgorithms.dll` + `VcgAlgorithms.lib` |
| Win32 输出 | 静态 `VcgAlgorithms.lib` |
| 命名空间 | `vcgalgo` |
| 依赖 | vcglib（头文件，`bin/SDK/vcglib`）、Eigen；MeshReconstruct 仅后处理 soup（Poisson 在 pclalgo） |
| 许可证 | GPL-3.0（vcglib） |

---

## 2. 数据契约

与 `PointCloudAlgorithm` 完全对齐：

| 缓冲 | 布局 |
|------|------|
| 网格 soup | `9*T` float，每三角 3 顶点 xyz（mm） |
| 索引化 mesh | `vertices: 3*N` float + `faces: 3*F` int |

---

## 3. API 总览

| 头文件 | 功能 |
|--------|------|
| `VcgMeshAdapter.h` | `IndexedMesh` 结构 + `triangleSoupToIndexedMesh` / `indexedMeshToTriangleSoup` |
| `MeshSimplify.h` | `simplifyQuadricEdgeCollapse` — quadric-error 边折叠简化 |
| `MeshSmooth.h` | `smoothLaplacian` / `smoothTaubin` / `applyMeshSmooth` |
| `MeshRepair.h` | `repairMesh` — 去重/退化/重复面/非流形/填孔 |
| `MeshRemesh.h` | `isotropicRemesh` — 各向同性重网格；`computeMedianEdgeLengthMm` — 边长中位数 |
| `MeshReconstruct.h` | `postProcessReconstructedMesh` — 对已有 soup 做简化/修复/平滑（Poisson 在 pclalgo） |
| `SelfTest.h` | `runSelfTest` |

### 3.1 网格简化

```cpp
vcgalgo::SimplifyParams params;
params.targetFaceCount = 10000;
params.qualityThreshold = 0.3;  // 0-1，越大质量越高
params.preserveBoundary = true;
params.preserveTopology = true;

std::vector<float> simplified;
vcgalgo::simplifyQuadricEdgeCollapse(soup, simplified, params);
```

### 3.2 网格平滑

```cpp
// Laplacian（快速，余切权重 + 边界保护）
vcgalgo::MeshSmoothParams lapParams;
lapParams.iterations = 3;
lapParams.useTaubin = false;
vcgalgo::applyMeshSmooth(soup, smoothed, lapParams);

// Taubin（保形，λ/μ）
vcgalgo::MeshSmoothParams taubinParams;
taubinParams.iterations = 3;
taubinParams.lambda = 0.2;
taubinParams.useTaubin = true;
vcgalgo::applyMeshSmooth(soup, fairing, taubinParams);
```

### 3.3 网格修复

```cpp
vcgalgo::RepairParams params;
params.removeDegenerate = true;
params.removeDuplicate = true;
params.removeDuplicateFaces = true;
params.fillHoles = true;
params.holeMaxEdgeCount = 30;

vcgalgo::RepairReport report;
vcgalgo::repairMesh(soup, repaired, params, &report);
```

### 3.4 各向同性重网格

```cpp
vcgalgo::isotropicRemesh(soup, 2.0, remeshed, 3);
// targetEdgeLengthMm = 2.0mm，iterations = 3
```

### 3.5 重建管线增强

```cpp
// 先 pclalgo::reconstructPoisson* 得 soup，再后处理：
vcgalgo::postProcessReconstructedMesh(
    soup, outSoup,
    50000,   // targetFaceCount（0=不简化）
    true,    // doRepair
    false    // doSmooth
);
```

**说明**：点云 Poisson/Scale-space 在 [`PointCloudAlgorithm`](../PointCloudAlgorithm/DEVELOPER_GUIDE.md)；本 API 只做 vcglib 后处理。**网格 → NURBS B-rep** 在 [`GeometryAlgorithm/MeshSurfaceReconstruction`](../GeometryAlgorithm/inc/MeshSurfaceReconstruction.h)。

---

## 4. 内部架构

```text
VcgAlgorithms.dll
├─ VcgMeshAdapter.cpp     — vcglib mesh 类型定义 + 数据转换
├─ VcgMeshTypes.h         — 内部头：vcg::Mesh 类型（不对外暴露）
├─ MeshSimplify.cpp       — vcg::LocalOptimization + QuadricEdgeCollapse
├─ MeshSmooth.cpp         — vcg::Smooth::VertexCoordLaplacian / ImplicitFairing
├─ MeshRepair.cpp         — vcg::tri::Clean + vcg::tri::Hole
├─ MeshRemesh.cpp         — vcg::tri::IsotropicRemeshing
├─ MeshReconstruct.cpp    — postProcessReconstructedMesh（简化/修复/平滑）
└─ SelfTest.cpp           — 立方体/球面测试用例
```

**关键设计**：vcglib 头文件仅在 `.cpp` 中 include，`.h` 不暴露任何 vcglib 类型。

---

## 5. 构建配置

### 5.1 vcxproj 配置

```xml
<!-- Additional Include Directories -->
inc;source;../../../../bin/SDK/vcglib;../../../../bin/SDK/eigen
../../../../bin/SDK/CGAL5.5.2/auxiliary/gmp/include;../../../../bin/SDK/CGAL5.5.2/include
../../PointCloudAlgorithm/inc

<!-- Preprocessor (x64) -->
VCg_ALGORITHMS_LIB;NOMINMAX

<!-- 输出 -->
$(CloudSimBinDir)VcgAlgorithms.dll
```

### 5.2 vcglib 编译注意事项

- vcglib 是头文件库，无 .lib 输出
- 需要 `/bigobj` 编译选项（模板展开产生大 obj）
- `NOMINMAX` 避免 Windows `min/max` 宏与 `std::min/max` 冲突
- vcglib `wrap/io_trimesh/` 拉入 OpenGL 依赖，本模块不使用

---

## 6. Data 集成

`PointCloudBackendOps.h` 新增方法（桩函数，待 VcgAlgorithms.dll 编译后实现）：

| 方法 | 说明 |
|------|------|
| `simplifyMesh` | 网格简化 |
| `smoothMeshLaplacian` | Laplacian 平滑 |
| `repairMesh` | 网格修复 |
| `remeshMeshIsotropic` | 各向同性重网格 |
| `analyzeMeshDefects` | 多信号缺陷检测（针状/突起/边界尖刺），只读报告 |
| `reconstructMeshFromPointCloudPoissonAndPostProcess` | 重建 + 后处理管线 |

---

## 7. 自检

```cpp
std::vector<std::string> failures;
const bool ok = vcgalgo::runSelfTest(failures);
```

测试用例：
1. IndexedMesh 转换往返（立方体去重验证）
2. 网格简化（球面简化到 1/3 面数）
3. Laplacian 平滑
4. Implicit Fairing 平滑
5. 网格修复
6. 各向同性重网格
7. 空输入处理
8. 缺陷检测（人造针状三角 cube+needle soup）

---

## 8. 相关文档

- [`PointCloudAlgorithm/DEVELOPER_GUIDE.md`](../PointCloudAlgorithm/DEVELOPER_GUIDE.md)
- [`GeometryEngine/DEVELOPER_GUIDE.md`](../GeometryEngine/DEVELOPER_GUIDE.md)
- [`ARCHITECTURE_SUMMARY.md`](../../ARCHITECTURE_SUMMARY.md)
- [`docs/vcglib_integration/`](../../docs/vcglib_integration/)
- vcglib 仓库：https://github.com/cnr-isti-vclab/vcglib
