# ACCEPTANCE — 算法工程整理

## 执行阶段

- [x] Align / Consensus / Design / Task
- [x] Automate：PointCloud Reconstruction 拆分 + 三模块公开头文档
- [x] Assess：本文件

## 验收对照 CONSENSUS

| 标准 | 结果 |
|------|------|
| `ReconstructionPoisson.h/.cpp` + `ReconstructionScaleSpace.h/.cpp` | 通过 |
| `Reconstruction.h` 仅 `#include` 转发 | 通过 |
| PointCloud 公开算法头：意图 + 参数默认/单位 + 失败条件 | 通过（Buffer/Parallel/SelfTest 短 brief） |
| Vcg 公开算法头同类补齐 | 通过 |
| Geometry 公开算法头同类补齐 | 通过（子代理 28 个头；内部 Registry/JSON 辅助跳过） |
| vcxproj / filters 更新；旧 `#include "Reconstruction.h"` 仍可用 | 通过 |
| DEVELOPER_GUIDE API 表同步 | 通过（PointCloud / Vcg） |
| 未改算法数值逻辑与默认参数值 | 通过 |

## 变更摘要

### PointCloudAlgorithm

- 拆分：`Reconstruction.cpp` → `ReconstructionPoisson.cpp` + `ReconstructionScaleSpace.cpp`
- 新头：`ReconstructionPoisson.h`、`ReconstructionScaleSpace.h`
- 兼容：`Reconstruction.h` 聚合转发
- 文档：Preprocess / Downsample / Measure / Transform / Crop / Registration* / Reconstruction* / Config / PointFeatures / Buffer

### VcgAlgorithms

- 文档：Simplify / Smooth / Repair / Remesh / Reconstruct / DefectDetect / NormalSmooth / Adapter
- GUIDE：`postProcessReconstructedMesh` 纠正旧文档中的 `reconstructAndPostProcess` 名称

### GeometryAlgorithm

- 公开算法头中文 Doxygen 补齐（签名未改）

## 已知保留

- `Preprocess` / `Downsample` 等多入口族文件本轮**不拆**（仅文档）
- `meshingRadiusMm` 仍未传入 CGAL（行为未改，文档已标明）
- Geometry 内部 Registry 辅助头未扩写
- 本机未强制全量编译自检（需 VS 构建 `PointCloudAlgorithm` / `VcgAlgorithms` / `GeometryAlgorithm` 验证）

## 建议验证

```text
1. 编译 PointCloudAlgorithm（含新 cpp）
2. 编译 Data（仍 include Reconstruction.h）
3. 可选：pclalgo::runSelfTest / vcgalgo::runSelfTest
```
