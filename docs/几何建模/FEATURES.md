# FEATURES — 几何建模已有功能清单

以代码为准：`GeometricModelingRibbonBar` 信号、`GeomodelingFeatureKind` / `GeomodelingExtrudeEnd`、`GeometryAlgorithm` 的 Sketch* 内核。

列说明：**入口** = Ribbon / 侧栏 / 树；**MVP 局限** = 已交付但仍弱，勿当成「未做」。

## 1. 工作区与持久化

| 功能 | 入口 | 关键文件 | MVP 局限 |
|------|------|----------|----------|
| 进入/退出几何建模 | 顶栏 `com.cloudsim.geomodeling` | `GeometricModelingPlugin.cpp` | 无顶层「进入」菜单 |
| Ribbon | 模式工具栏 | `GeometricModelingRibbonBar.*` | — |
| 特征树（编辑/隐藏/删除/回退/面上草图） | 左侧栏 | `GeometricModelingPage.*` | PM 未对齐 SW；**命名参数页 MVP 已有** |
| Undo / Redo | Ribbon | `CommandStack.*`、`BodyHistoryCmd.h` | — |
| **命名参数面** | 侧栏 Params | Page `showNamedParams` + Session `applyNamedParam` | 圆/线/椭圆 + Pad 深度/拔模角 |
| 工程侧车 + Body history 同步 | 存盘/加载钩子 | Plugin + `FeatureDocument` + Data | 侧车多为 UI 态（如 `activeBodyId`） |
| 原点 / XY·XZ·YZ 可见性 | 树 + Host | `setOriginReferenceVisibility` | — |
| 多 Body / Pocket 目标 | 侧栏 Body 下拉 | Page + extrude params | — |

## 2. 草图绘制与编辑

| 功能 | 入口 | 关键文件 | MVP 局限 |
|------|------|----------|----------|
| 新建草图（面 / 原点平面 / 用户参考面） | Ribbon 新建 | Plugin `onNewSketch`、Host `pickSketchSupportPlane` | 需 Host 1.49.0+ |
| 基准面 DatumPlane | Ribbon | Plugin `onDatumPlane`、FeatureDocument | 等距可从基面或模型面；源引用+偏移可编；草图可挂 `datumPlaneId` |
| 结束草图 | Ribbon | SketchEditSession | — |
| 直线 / 圆弧 / 圆 / 矩形 | 绘制组 | `SketchTools.*`、`SketchEditSession.*` | — |
| 椭圆 | 绘制组 | 同上 | 已进 PlaneGCS；长短轴约束/尺寸可用 |
| 正多边形（3–24） | 绘制组 | 同上 | — |
| 直槽口 Slot | 绘制组 | 同上 | — |
| 样条 Spline | 绘制组 | 同上 | **双模式** Through/Control；无整段 BSpline GCS |
| 构造线 | 绘制组 | SketchTools | — |
| 修剪 / 镜像 / 删除 | 编辑 | SketchEditSession / SketchGeom | — |
| 求解 + DOF | Ribbon 求解 | `SketchConstraintSolver.*`、`third_party/planegcs` | — |
| Overlay / 屏→面映射 | Host | `beginSketchInput`、`setSketchOverlay`、`mapScreenToSketchPlane` | — |

## 3. 尺寸与几何约束

| 功能 | 入口 | 关键文件 | MVP 局限 |
|------|------|----------|----------|
| 长度 / 距离 / 半径 / 角度 / 弧半径 | 标注组 | SketchTools、SketchGeom | 半径工具可打椭圆长半轴（MajorRadius） |
| 水平 / 竖直 / 重合 / 平行 / 垂直 | 约束组 | 同上 + PlaneGCS | — |
| 等长 / 相切 / 对称 / 中点 | 约束组 | 同上 | — |
| 固定 / 到原点 | 约束组 | 同上 | — |

## 4. 草图引用

| 功能 | 入口 | 关键文件 | MVP 局限 |
|------|------|----------|----------|
| 投影边 | Ribbon | `GeometricModelingSolidFeatures.cpp` | — |
| 转换实体 Convert | Ribbon | 同上 | 圆/弧优先；折线回退有统计警告 |
| 等距 Offset | Ribbon | 同上 | 孔环/尖角仍可能失败 |

## 5. 实体特征

| 功能 | Kind | 入口 | 算法 / Host | MVP 局限 |
|------|------|------|-------------|----------|
| 拉伸 Pad | `Pad` | 侧栏 Extrude | `SketchExtrude` → `extrudeSketchProfileToBrep` | **startOffset** + **TwoDirections**（ABI 1.48）；MidPlane 仍对称 |
| 切除 Pocket | `Pocket` | 同上 | 同上 | — |
| 终止：Blind / UpToFace / MidPlane / TwoDirections / ThroughAll / UpToVertex / OffsetFromFace | `GeomodelingExtrudeEnd` | 侧栏 | rebuild | UpToVertex 取边折线首点类弱引用 |
| 拉伸拔模角 | Pad/Pocket 参数 | 侧栏 | Extrude | — |
| 扫描 / 扫描切除 | `Sweep` / `SweepCut` | 侧栏 Sweep | `SketchSweep` | **扭转=轮廓预旋转**，非 PipeShell 律；路径仍偏 combo |
| 模型边作路径 | Sweep 参数 | 侧栏 / 点选 | SolidFeatures | MVP 边→折线 |
| 圆角 / 倒角 | `Fillet` / `Chamfer` | 侧栏 | `SketchFillet` | 智能选边未做 |
| 旋转 / 旋转切除 | `Revolve` / `RevolveCut` | 侧栏 | `SketchRevolve` | — |
| 线性阵列 | `LinearPattern` | 侧栏 | `SketchPattern` | **特征贡献 seed**（after CUT before）+ fuseOnto |
| 圆周阵列 | `CircularPattern` | 侧栏 | `SketchPattern` | 轴边拾取；seed 与线性共用 tip 贡献语义 |
| 镜像 3D | `Mirror3D` | 侧栏 | SketchPattern (mirror) | — |
| 放样 / 放样切除 | `Loft` / `LoftCut` | 侧栏 | `SketchLoft` | — |
| 抽壳 | `Shell` | 侧栏 | `SketchShell` | — |
| 独立拔模 | `Draft` | 侧栏 | `SketchDraft` | 中性面能力有限 |
| 重建 Rebuild | — | Ribbon | Host history JSON | — |

## 6. 特征 kind 枚举对照

`FeatureDocument.h` → `GeomodelingFeatureKind`：

`Sketch` · `Pad` · `Pocket` · `Sweep` · `SweepCut` · `Fillet` · `Chamfer` · `Revolve` · `RevolveCut` · `LinearPattern` · `CircularPattern` · `Mirror3D` · `Loft` · `LoftCut` · `Shell` · `Draft` · `DatumPlane` · `DatumPlaneAngle`

`DatumPlane` 仅 FeatureDocument（可持久化 + overlay），不进 Parametric tip。

## 7. 已交付但勿写成「待做」

| 项 | 状态 | 来源 |
|----|------|------|
| 椭圆/多边形/槽口、Offset、Convert、扫描边路径+扭转 MVP、UpToVertex/OffsetFromFace、Draft | 已交付 | `docs/SW差距续期/` |
| Convert 真面、Offset 孔环、多边形 3–24、DatumPlane 持久化/overlay、特征级 LinearPattern | 已交付 | `docs/硬化基准面/` |
| AI → Parametric `feature.compose` | 已交付 | `docs/特征史AI/` |
| 命名参数面 MVP、椭圆 GCS、Convert 保型、样条双模式 | 已交付 | `docs/草图硬化/` |
| JSON 导入导出 / Compose 文件 / `cloudsim_geom` | 已交付 | `docs/脚本建模/` |

下一期增量见 [ROADMAP.md](ROADMAP.md)。
