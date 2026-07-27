# CONSENSUS — 算法工程整理

## 1. 需求与验收

整理 `GeometryAlgorithm` / `PointCloudAlgorithm` / `VcgAlgorithms` 公开算法 API：

1. **族级文件**为主；仅拆明显混杂算法（本轮明确：`Reconstruction` → Poisson / Scale-space）
2. **旧头保留**为 `#include` 转发，兼容 Data 等既有 include
3. **执行顺序**：先 PointCloud + Vcg，再 Geometry
4. 头文件文档须含：**算法意图** + **参数默认/单位** + **失败条件**

### 验收标准

- [ ] `ReconstructionPoisson.h` / `ReconstructionScaleSpace.h` + 对应 `.cpp` 存在；`Reconstruction.h` 仅转发
- [ ] PointCloud / Vcg 公开算法头均具备上述三类说明（工具/配置头可精简）
- [ ] Geometry 公开算法头完成同类补齐
- [ ] `*.vcxproj` / filters 已更新；既有 `#include "Reconstruction.h"` 仍可编译
- [ ] 三份 `DEVELOPER_GUIDE.md` API 表路径同步
- [ ] 不改算法数值逻辑与默认参数值

## 2. 技术方案

| 项 | 约定 |
|----|------|
| 文档风格 | 中文 Doxygen：`@file`/`@brief` + 函数块注释；参数行注释写默认与单位 |
| 拆分边界 | 仅 Poisson ↔ Scale-space；`Preprocess`/`Downsample` 等本轮只补文档不拆 |
| 兼容 | `Reconstruction.h` → `#include` 两个新头；符号仍在 `pclalgo` |
| 编码 | UTF-8 BOM + CRLF（事后 `normalize_source_encoding.py`） |
| 实现约束 | 禁止改 CGAL/vcglib 调用语义；禁止无谓重构 |

## 3. 任务边界

**做**：公开头文档；Reconstruction 拆分；vcxproj；DEVELOPER_GUIDE API 行；ACCEPTANCE。

**不做**：改配准/重建数值；拆 Tubular/MeshSurface 内部；改 Data 业务逻辑（除必要时 include）。

## 4. 已确认决策

- Q1=A 族级 + 拆混杂
- Q2=B 旧头转发
- Q3=A 先 PointCloud+Vcg 再 Geometry
- Q4=意图+参数表+失败条件
