# 几何建模（活跃文档）

独立 SDK 插件 `com.cloudsim.geomodeling` 的开发入口。代码真源以插件 Ribbon / `FeatureDocument` / Host ABI 为准；本目录描述**现状**与**下一期路线图**。

## 现状摘要

- 宿主顶栏工作区切换进入（非顶层菜单）；Host **≥ 1.44.0**
- 草图：PlaneGCS 约束求解 + 多种绘制/标注/引用工具
- 实体：Pad/Pocket、Sweep、Fillet/Chamfer、Revolve、Pattern、Mirror、Loft、Shell、Draft、DatumPlane
- Body 落在 `ParametricBrepModel`；特征历史经 Host/Data rebuild
- **网页端**：同一 rebuild，经 `HeadlessGeomodelBridge` / `/api/geomodeling/*`；实体特征（含扫描/放样）、撤销重做、特征史导入导出已进 `GeomodelingShell`。PlaneGCS 视口草图与拾取仍仅桌面。对照见 [CloudSim/README.md](../../README.md)「几何建模：桌面逻辑 → 网页」；清单 §8 见 [FEATURES.md](FEATURES.md)

## 本目录

| 文档 | 内容 |
|------|------|
| [FEATURES.md](FEATURES.md) | 已有功能清单（对 Ribbon / FeatureKind / 算法） |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 分层架构与数据流 |
| [ROADMAP.md](ROADMAP.md) | 已知债 + 四包下一期落地方案 |

插件短说明：[GeometricModelingPlugin/README.md](../../src/Plugins/GeometricModelingPlugin/README.md)

## 相关专题（勿重复抄写）

| 路径 | 说明 |
|------|------|
| [后端对象与软件模式/](../后端对象与软件模式/) | Parametric Body 与侧车键 |
| [_archive/草图硬化/](../_archive/草图硬化/) | 命名参数面 + 椭圆 GCS + Convert 保弧 + 样条双模式 |
| [_archive/SW差距续期/](../_archive/SW差距续期/) | 椭圆/Offset/Convert/扫描硬化/终止条件/Draft 等已交付 |
| [_archive/硬化基准面/](../_archive/硬化基准面/) | Convert/UpToVertex/Offset/DatumPlane/特征级 Pattern 等 |
| [_archive/特征史AI/](../_archive/特征史AI/) | `feature.compose` → Parametric + 特征树 sync |
| [_archive/WorkspaceModeSwitcher/](../_archive/WorkspaceModeSwitcher/) | 顶栏主程序/几何建模/工艺/工程图切换 |
| [_archive/几何建模/](../_archive/几何建模/) | 一期 6A 历史（CONSENSUS 等；部分表述已过时） |

## 构建提示

Debug|x64 与 Release|x64 均须通过；产物分别在 `bin/x64d` 与 `bin/x64` 的 `plugins/com.cloudsim.geomodeling/`。
