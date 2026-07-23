# ALIGNMENT — 算法工程整理

## 1. 项目上下文

| 模块 | 输出 | 命名空间 | 现状 |
|------|------|----------|------|
| `GeometryAlgorithm` | DLL | `geoalgo` | 公开头已按功能拆分；复杂流水线（Tubular / MeshSurface）已有子目录实现 |
| `PointCloudAlgorithm` | 静态库 | `pclalgo` | 公开头按能力拆分（配准/重建/预处理等），多数仅有 `@brief Xxx 接口` |
| `VcgAlgorithms` | DLL | `vcgalgo` | 已一对一：Simplify / Smooth / Repair / Remesh / …；`MeshNormalSmooth.h` 文档相对完整 |

数据契约统一：点云 `3*N` float（mm）、三角 soup `9*T` float；本库不持有 `worldMatrix`。

编码约定：UTF-8 BOM + CRLF；中文简练注释；`CONVENTIONS.md` / `SOURCE_CONVENTIONS.md`。

## 2. 原始需求

整理上述三份 `DEVELOPER_GUIDE.md` 覆盖的算法工程：

1. **每种算法单独的头文件和 cpp**
2. **头文件添加算法说明和参数说明**

## 3. 边界确认

### 纳入

- 三模块**对外公开** `inc/*.h` 的算法 API 文档补齐（算法意图、入参/出参、默认值、单位、失败条件）
- 仍捆在同一文件内的**可独立命名**算法，按确认粒度拆成独立 `.h` / `.cpp`
- 拆分后更新对应 `*.vcxproj` / filters；必要时保留**兼容聚合头**（`#include` 转发）以免破坏 Data/Host 现有 include
- 同步更新三份 `DEVELOPER_GUIDE.md` 的 API 总览表（路径变更处）

### 不纳入（除非另决策）

- 改算法数值逻辑、调参默认值
- 重写 `DEVELOPER_GUIDE` 长文流水线章节（只改 API 索引与路径）
- 改 Data / Plugin / Host 业务逻辑（除 include 路径适配）
- 内部实现细节头（如 `source/spare/*`、`TubularGrindingCommon.h`）除非随公开 API 拆分必须调整
- SelfTest / 全局导出宏 / 适配器类本身不当作「算法」拆文件

## 4. 需求理解

当前痛点主要是：

1. **文档空洞**：大量公开头只有 `/// @brief Xxx 接口`，参数无说明，开发者只能翻 `DEVELOPER_GUIDE` 或 `.cpp`
2. **粒度不齐**：Vcg 已基本一算法一族一文件；PointCloud 的 `Preprocess` / `Reconstruction` / `Downsample` 等仍多算法共文件；Geometry 公开门面已拆，大流水线实现已在子目录

目标形态（对齐 `MeshNormalSmooth.h` / `TubularGrindingParams` 风格）：

```cpp
/// @file ReconstructionPoisson.h
/// @brief Poisson 隐式表面重建：定向点云 → 水密倾向三角 soup（mm）

/**
 * CGAL poisson_surface_reconstruction_delaunay
 * @param xyz 3*N float，mm
 * @param normalsNxNyNz 3*N，与 xyz 同序；方向错误会导致翻面/空洞
 * @param spacingMm ≤0 时用平均点距；控制八叉树尺度
 * ...
 */
bool reconstructPoisson(...);
```

## 5. 疑问澄清（待决策）

### Q1. 「每种算法」粒度（最高优先级）

| 方案 | 含义 | 影响 |
|------|------|------|
| **A. 算法族**（推荐默认） | 保持现有文件边界，重点补文档；仅拆明显混杂文件（如 Poisson / Scale-space 分文件） | 改动可控，API 路径变化少 |
| **B. 入口函数级** | 每个导出函数一对 `.h/.cpp`（如 `EstimateNormalsPca.h`、`RemoveOutliers.h`） | 文件爆炸，include 与 vcxproj 大改 |
| **C. 混合** | 对外保留族级聚合头；内部实现可更细（可选） | 文档与兼容兼顾，工作量中等 |

### Q2. 拆分后旧头文件

- **A.** 删除旧头，调用方改 include（Breaking）
- **B.** 旧头保留为 `#include` 聚合转发（推荐，兼容 Data）

### Q3. 执行范围与顺序

- **A.** 先 `PointCloudAlgorithm` + `VcgAlgorithms`（小、边界清），再 `GeometryAlgorithm`
- **B.** 三模块并行一次做完
- **C.** 仅补全三模块头文件文档，**暂不拆文件**

### Q4. 文档内容深度

头文件说明是否需要把 `DEVELOPER_GUIDE` 中的流程/调参表**精简迁入**（推荐：算法一句 + 参数表 + 失败条件），还是仅短 `@brief` + `@param`？

## 6. 初步建议（未确认前不实施）

若无额外偏好，倾向：

1. **Q1 = A**：族级为主；明确拆：`Reconstruction`→Poisson / ScaleSpace；`Preprocess` 可按「法线 / 离群平滑 / 重建前管线」再议
2. **Q2 = B**：兼容聚合头
3. **Q3 = A**：先点云+Vcg，再 Geometry
4. **Q4**：中文 Doxygen，含算法意图 + 参数默认/单位 + 关键限制（不对齐粘贴整章流水线）

---

**状态**：Align 中断 — 等待 Q1–Q4 决策后写 `CONSENSUS` 并进入 Architect。
