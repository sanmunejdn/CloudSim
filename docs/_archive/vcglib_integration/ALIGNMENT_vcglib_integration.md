# vcglib 集成 - 对齐文档

## 1. 原始需求

在 `bin/SDK` 部署 vcglib，评估升级点云重构 mesh 的库方案，决定封装还是直接使用。

## 2. 项目上下文

### 2.1 现有架构

- `PointCloudAlgorithm`：静态库，链入 `Data.dll`，命名空间 `pclalgo`
- 数据契约：`std::vector<float>` xyz（3*N, mm）、triangleSoup（9*T, float）
- 现有重建：CGAL Poisson + Scale-space（`Reconstruction.h`）
- 现有优化：TBB 并行化、`ReconstructionConfig` 质量分级
- 插件层：`PointCloudPlugin` 通过 `IPluginPointCloudHost` 访问点云能力

### 2.2 vcglib 概况

- 仓库：https://github.com/cnr-isti-vclab/vcglib.git
- 部署位置：`bin/SDK/vcglib/vcg/`
- 类型：C++ 模板头文件库，无外部依赖
- 许可证：GPL-3.0
- 核心能力：网格简化、平滑、修复、重网格化、空间查询、I/O

## 3. 需求理解

### 3.1 能力缺口分析

| 当前缺失能力 | vcglib 提供 | 业务价值 |
|-------------|------------|---------|
| 网格简化 | quadric-error edge collapse | 重建后控制面数，适配下游渲染/仿真 |
| 网格平滑 | Laplacian / implicit fairing | 去噪、改善网格质量 |
| 网格修复 | 去重/退化面/孔洞填充 | 重建后自动修复拓扑问题 |
| 各向同性重网格 | isotropic remeshing | 均匀三角形分布，提升仿真精度 |
| 重建管线增强 | Poisson 后自动简化+修复 | 一键从点云到可用 mesh |

### 3.2 vcglib 不提供的能力

- Poisson 表面重建（需保留 CGAL）
- Scale-space 重建（需保留 CGAL）

## 4. 边界确认

### 4.1 包含范围

- 新建 `VcgAlgorithms` 模块（独立 DLL）
- 网格后处理：简化、平滑、修复、重网格
- 重建管线增强：CGAL 重建 → vcglib 后处理
- Plugin SDK 暴露新能力

### 4.2 不包含范围

- 替换 CGAL 重建算法
- vcglib I/O 替换现有 mesh I/O
- vcglib 空间查询替换 CGAL 空间结构
- GPU 加速

## 5. 疑问澄清

### 5.1 已确认

- [x] 许可证：GPL-3.0 可接受
- [x] 模块位置：新建独立 DLL（非嵌入 PointCloudAlgorithm）
- [x] 集成范围：Phase 1+2 全部

### 5.2 待确认

- [ ] `VcgAlgorithms` 是否需要暴露给 Plugin SDK？（建议：Phase 2 再考虑）
- [ ] 重建后默认简化目标面数？（建议：可配置，默认保留重建面数）
- [ ] 是否需要 vcglib 的 mesh I/O 能力？（建议：暂不需要，现有 I/O 已够用）

## 6. 参考文档

- [文档索引](../../README.md) §4.3, §5
- [`PointCloudAlgorithm/DEVELOPER_GUIDE.md`](../../src/Geometry/PointCloudAlgorithm/DEVELOPER_GUIDE.md)
- [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../../src/Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md)
- [`mesh_reconstruction_optimization/`](../mesh_reconstruction_optimization/)
- vcglib 仓库：https://github.com/cnr-isti-vclab/vcglib
