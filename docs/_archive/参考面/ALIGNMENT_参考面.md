# ALIGNMENT — 参考面（偏移面）草图支撑

## 原始需求

> 设计参考面功能：现在草图只能选基面和模型面；希望基于基面或模型面偏移创建参考面，再在该面上创建草图。

## 项目现状（以代码为准）

| 能力 | 状态 | 位置 |
|------|------|------|
| Ribbon「基准面」 | **已有** | `onDatumPlane`：等距面 / 三点 / 成角面 |
| 等距创建 | **已有（仅模型平面面）** | 点选 Face → `queryFaceSketchPlane` → `offsetPlaneAlongNormal` → `FeatureDocument::addDatumPlane` |
| 成角 / 三点 | **已有** | 同文件；铰链用边向 |
| 树显示 + overlay | **已有** | FeatureDocument 侧车；不进 Parametric tip |
| 树上双击参考面开草图 | **已有** | `featureEditRequested` → `beginSketchOnPlane(feature->plane)` |
| Ribbon「新建草图」点选 | **仅基面 + 模型面** | Host `pickOriginSketchPlane`：`beginOriginPlaneSelection` + MeshFace；**不拾取用户 Datum overlay** |
| 等距相对 XY/XZ/YZ 基面 | **缺口** | 等距模式只 `pickStepElement(Face)`，不能点选原点平面 |
| 关联驱动（父面移动后重算） | **未做** | FEATURES：无关联驱动 / 二次编辑 PM |
| 草图 ↔ 参考面引用 | **无 id** | Sketch 只存烘焙的 `PluginSketchPlane` 帧，无 `datumPlaneId`；面/偏移变更不会回写草图 |
| 一键「偏移后直接开草图」 | **无** | 现流程为两步：创建 Datum → 树编辑/双击开草图 |
| GeometryAlgorithm 建面 API | **无** | 等距/三点/旋转数学在插件侧；GA 仅有 `queryPlanarFaceSketchPlane` |
| 脚本/Parametric tip | **不含 Datum** | SCHEMA：DatumPlane 仅 FeatureDocument 侧车 |

结论：需求中的「偏移创建面」大部分已交付；痛点更可能是 **新建草图拾取链路不含用户参考面**，以及 **无法从基面等距** / UX 不够直观。探索核对见 [Explore datum plane code](47f3eb28-4ab1-427b-8ef3-1dad5efc67d0)。

## 边界确认（待共识）

**拟纳入（默认最小集，待你确认）：**

1. 新建草图时可点选已创建的参考面（视口），不仅靠特征树双击。
2. 等距创建时允许参考源 = 原点基面（XY/XZ/YZ）或模型平面面。
3. 创建后可在该参考面上开草图（树双击保持；新建草图拾取补齐）。

**拟不纳入（除非你点名）：**

- SolidWorks 级属性管理器二次编辑、多约束参考面（点+线、平行面等全套）
- 参考轴 / 坐标系
- 关联驱动（父几何变更自动重算偏移面）
- 把 DatumPlane 写入 Parametric tip（当前侧车策略不变）

## 需求理解

用户目标闭环：

```text
选参考源(基面|模型面) → 输入偏移 → 得到 DatumPlane → 在其上 New Sketch / 双击开草图
```

现有已覆盖「模型面 → 偏移 → 树双击开草图」；缺「基面作源」与「新建草图视口点选 Datum」。

## 候选方向（未定案）

| 方案 | 做法 | 优点 | 代价 |
|------|------|------|------|
| A. 补齐拾取（推荐） | 插件在 `onNewSketch` 并行命中 Datum overlay；等距源增加原点平面 | 改动面小，复用现有 Datum | 需 Host/插件约定命中用户平面 |
| B. 仅文档/引导 | 教用户用「基准面」+ 树双击 | 零开发 | 不解决「新建草图选不到」 |
| C. 大改参考面系统 | 关联式 Datum + PM 面板 + tip 重建 | 长期对齐 SW | 范围大，超出本需求 |

推荐 **A**：在现有 DatumPlane 上补缺口，而不是重做一套「参考面」。

## 需求理解确认

用户选择：**C + D**（见 `CONSENSUS_参考面.md`）。

- **P1**：新建草图可点选用户参考面 + 等距可从原点基面创建  
- **P2**：源引用持久化、侧栏二次编辑、重建时关联重算  

## 疑问澄清（按优先级）

1. **草图跟随深度（D-S0 vs D-S1）** — 实现 P2 前必须确认（见 CONSENSUS 未决）。
2. ~~本期优先缺口~~ → 已确认 C+D。
3. 脚本/JSON compose 声明偏移源：P2 仅 FeatureDocument 侧车即可，不强制进 tip。

## 关键文件

- `GeometricModelingPlugin.cpp` — `onNewSketch` / `onDatumPlane` / `beginSketchOnPlane`
- `PluginGeometryHostImpl.cpp` — `pickOriginSketchPlane`
- `FeatureDocument.*` — `DatumPlane` / `DatumPlaneAngle`
- 既有文档：`docs/硬化基准面/`、`docs/几何建模/FEATURES.md`
