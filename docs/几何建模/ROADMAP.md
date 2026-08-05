# ROADMAP — 已知债与里程碑

## 官方优先栈（锁定）

| 序 | 能力 | 状态 |
|----|------|------|
| 1 | 命名参数面 MVP | **已交付**（`docs/_archive/草图硬化/`） |
| 2 | 椭圆 GCS / Convert 保圆弧 / 样条双模式 | **已交付**（包 A） |
| 3 | 拉伸 startOffset·双向深度、圆周阵列、成角基准面 | **已交付**（`docs/_archive/特征阵列与拉伸深化/`，Host ABI **1.48.0**） |
| 4 | TopoNaming 自研渐进 | **下期独立专题**（见 `docs/TopoNaming/`） |
| — | JSON / Python 脚本建模 | **已交付**（`docs/_archive/脚本建模/`；一期无 ABI bump） |

参考 Yi3D 语义，不移植 Transaction/Element。

## 1. 仍存 MVP 债

| 项 | 说明 |
|----|------|
| Offset | 复杂尖角 / 孔环边界仍可能失败 |
| 扫描扭转 | 轮廓预旋转，非 PipeShell 律 |
| Vertex | 边端点吸附，非 TopExp 顶点索引 |
| 视口点选 | Sweep 等仍偏 combo |
| 命名参数面 | 仅圆/线/椭圆 + Pad 深度；未铺全特征 |
| 样条 | 控制点模式无 OCC 真 BSpline；无整段 GCS |
| 成角基准面 | 已交付铰链边向；无关联驱动 / 二次编辑 PM |

## 2. 明确不做

装配、曲面草图、工程图双向尺寸、完整 FreeCAD/SW 拓扑命名引擎、薄壁引导线扫描、Rib/孔向导（除非单独立项）。

## 3. 本期已交付 — 包 B + C 子集（优先栈第 3 项）

详见 `docs/_archive/特征阵列与拉伸深化/`：

| 项 | 状态 |
|----|------|
| Pattern tip 特征贡献 seed | **已交付**（`tipBefore` + Cut） |
| 成角基准面铰链边向 | **已交付** |
| 圆周阵列 | **已交付**（全栈 + tip 共用） |
| 拉伸 startOffset + TwoDirections | **已交付**（ABI 1.48.0） |
| PipeShell 扭转 / 引导线 | **未做**（可再拆） |

## 4. 下期 — 包 D + TopoNaming 专题（优先栈第 4 项）

| 阶段 | 内容 |
|------|------|
| 包 D 起步 | 视口点选硬化、特征 Suppress；可与 TopoNaming P0 并行 |
| TopoNaming 专题 | 见 [`docs/TopoNaming/ALIGNMENT_TopoNaming.md`](../TopoNaming/ALIGNMENT_TopoNaming.md)；弱命名表 → rebuild 映射 → Host API |

## 4b. 脚本建模（已交付）

见 `docs/_archive/脚本建模/`：Ribbon 导出/导入 history、运行 compose；进程内 `cloudsim_geom` + 控制台。不做 headless。

## 5. 包 A 交付摘要（已完成）

见 `docs/_archive/草图硬化/ACCEPTANCE_草图硬化.md`：A0 命名参数、A1 椭圆 GCS、A2 Convert 保弧、A3 样条双模式；Debug+Release 已编过。
