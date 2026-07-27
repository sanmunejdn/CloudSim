# CONSENSUS — 几何建模 SDK 插件

## 需求描述

独立插件提供 SolidWorks 式特征建模入口：菜单进入独占工作区（仅 AI + 中央建模页），中区嵌入 3D，选面创建约束草图，Pad/Pocket 生成/裁剪 B-rep，特征树参数化重建 + Undo/Redo。

## 验收标准

1. 菜单进入/返回；无全局左右栏；仅 AI + 建模页；中区可交互 3D
2. 选平面 Face 建草图；约束求解；overlay 贴面
3. Pad/Pocket → `BrepModel` 可见；改参 rebuild
4. Undo/Redo ≥ 20 步；工程存读一致
5. 插件仅链 PluginSDK；许可与 ORIGIN.md 齐全

## 技术方案

| 层 | 方案 |
|----|------|
| 求解器 | vendor FreeCAD/planegcs（LGPL） |
| 拉伸核 | 复制 FreeCAD FeatureExtrude OCC 段 → `geoalgo::sketchExtrude` |
| 插件主体 | 复制 OneCAD 草图/特征/Undo 思路与可移植源，改壳接 Host |
| 布局 | ProcessFlow 同序 + `embedActiveRenderWidget` |
| 持久化 | `onProjectAboutToSave` / `onProjectLoaded` + B-rep sidecar |

## 技术约束

- 插件不链 GeometryAlgorithm/Data/OSG
- Host vtable 仅追加；`minHostVersion` 对齐
- 禁止 GPL（dune3d / SolveSpace）源码复制

## 任务边界

一期原范围不做：曲面草图、圆角、完整拓扑命名。

**实现状态（相对原文）：** UpToFace、多 Body、MidPlane/ThroughAll、UpToFace 活引用、面归属已落地。完整拓扑命名、曲面草图、圆角、开曲面仍排除。实体扫描 MVP 见 `CONSENSUS_扫描特征.md`。
