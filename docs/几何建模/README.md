# 几何建模（活跃文档）

独立 SDK 插件 `com.cloudsim.geomodeling` 的开发入口。代码真源以插件 Ribbon / `FeatureDocument` / Host ABI 为准；本目录描述**现状**与**下一期路线图**。

## 现状摘要

- 宿主顶栏工作区切换进入（非顶层菜单）；Host **≥ 1.44.0**
- 草图：PlaneGCS 约束求解 + 多种绘制/标注/引用工具
- 实体：Pad/Pocket、Sweep、Fillet/Chamfer、Revolve、Pattern、Mirror、Loft、Shell、Draft、DatumPlane
- Body 落在 `ParametricBrepModel`；特征历史经 Host/Data rebuild

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
| [草图硬化/](../草图硬化/) | 命名参数面 + 椭圆 GCS + Convert 保型 + 样条双模式 |
| [SW差距续期/](../SW差距续期/) | 椭圆/Offset/Convert/扫描硬化/终止条件/Draft 等已交付 |
| [硬化基准面/](../硬化基准面/) | Convert/UpToVertex/Offset/DatumPlane/特征级 Pattern 等 |
| [特征史AI/](../特征史AI/) | `feature.compose` → Parametric + 特征树 sync |
| [WorkspaceModeSwitcher/](../WorkspaceModeSwitcher/) | 顶栏主程序/几何建模/工艺/工程图切换 |
| [后端对象与软件模式/](../后端对象与软件模式/) | Parametric Body 与侧车键 |
| [_archive/几何建模/](../_archive/几何建模/) | 一期 6A 历史（CONSENSUS 等；部分表述已过时） |

## 构建提示

Debug|x64 与 Release|x64 均须通过；产物分别在 `bin/x64d` 与 `bin/x64` 的 `plugins/com.cloudsim.geomodeling/`。
