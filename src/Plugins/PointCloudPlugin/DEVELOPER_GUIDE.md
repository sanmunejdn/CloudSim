# PointCloudPlugin 示例

> **文档导航**：[全库入口](../../README.md) · [全量目录](../../README.md) · [开发手册](../../开发手册/01-总览.md) · [产品索引](../README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

点云处理插件，演示 **1.2.0+** SDK：`IPluginPointCloudHost` + 侧栏 UI。

## 构建与部署

| 项 | 说明 |
|----|------|
| 工程 | `PointCloudPlugin.vcxproj`（x64，v142，Qt 5.14.2） |
| 链接 | **仅** `CloudSimPluginSDK.lib` |
| 部署 | `bin/x64(d)/plugins/com.cloudsim.pointcloud/plugin.json` + `PointCloudPlugin.dll` |
| `minHostVersion` | `"1.53.0"`（几何金字塔 + SDF/DDF + SPARE + 分割标注宿主 API） |

## 运行时

- 侧栏 Tab **点云** / **Point Cloud**：导入、列表、下采样、裁剪（包围盒/球/多边形）、预处理、**ICP / SPARE / SDF·DDF / 几何金字塔配准**、重建
- 菜单 **Tools → Point Cloud**（中文下子菜单标题为 **点云**）
- 管状铸件算法仍在 `GeometryAlgorithm/TubularGrinding`（含 SelfTest）；Plugin SDK / 特征构建侧栏 / AI `feature.build` 已拆除
- 语言：默认中文；切换 **设置 → Language → 中文/English** 时侧栏与菜单同步更新
- **重建网格 → 导出 PLY**：侧栏「重建网格」区选网格对象，点 **导出 PLY…**（或菜单 **Tools → 点云 → 导出网格 PLY…`）

**多边形裁剪（1.11.0+）**（侧栏「裁剪」区）：

1. 选中点云 → 选择 **保留内部** / **删除内部**
2. **多边形裁剪…** → `pickPolylineFromViewport` 进入 3D 绘制（左键顶点、右键/双击闭合、Esc 取消）
3. 闭合后自动 `cropPointCloudByPolyline`（屏幕投影 + 点在多边形内判断）

典型流程：

1. `importFileIntoActiveDocument(path, true)` 得 `backendId`
2. `doc->queryPointCloudInfo(id, info)` 显示点数
3. `host->pointCloudHost()->downsamplePointCloudVoxel(doc, id, params, onFinished)`
4. `onFinished` 中刷新列表；场景由宿主自动 `loadPointCloudFromBackendData`

**模板 B-rep 更新**（侧栏「CAD 模板 B-rep 更新」区）：

1. 导入 STEP → `BrepModel`、扫描 PLY → 点云，或 Poisson/导入 → `Model` 网格
2. 3D 视图手动对齐点云/网格与 CAD
3. 侧栏「CAD 模板 B-rep 更新」：**扫描数据** 下拉选点云或网格；选择模板 B-rep；可选 **选择面…**（`geometryHost()->pickStepElementFromViewport`）累积面索引，空列表=全部面
4. **匹配 (ICP)** → `registerScanToCadTemplate`
5. **面重构** → `updateTemplateBrepFromAlignedScan`（须先匹配）；详见 `docs/ARCHIVE_ZIP_LOCATION.txt`

## SPARE 非刚性配准（1.16.0+）

原理与调参通俗说明见 `docs/ARCHIVE_ZIP_LOCATION.txt`。

侧栏「配准」区：**方法** 下拉可选 **刚性 ICP**、**SPARE 非刚性**（**1.16.0+**）、**SDF/DDF 非刚性**（**1.17.0+**）或 **几何金字塔**（**1.53.0+**）。

| 控件 | 说明 |
|------|------|
| 非刚性源 / 目标 | 独立下拉，列出点云与网格（SPARE / SDF / 金字塔共用；金字塔限网格） |
| SPARE 选项 | 体素预滤波 (mm)、刚性预对齐、输出为新对象 |
| 执行 SPARE | `nonRigidRegisterSpare(...)` |

典型流程：

1. 方法选 **SPARE**；在「非刚性源 / 目标」下拉中分别选点云或网格（须为不同对象）
2. 可选体素预滤波、刚性 ICP 预对齐
3. 勾选「输出为新对象」则生成 `*_SPARE` / `*_spare` 后缀对象
4. 回调 `PluginPointCloudJobResult`：`rmseMm`（平均对称点-面误差）、`spareDeformationNodeCount`

算法与参数映射见 [`PointCloudAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/PointCloudAlgorithm/DEVELOPER_GUIDE.md) §3.5。

## SDF/DDF 混合非刚性配准（1.17.0+）

原理见 `docs/ARCHIVE_ZIP_LOCATION.txt`。

| 控件 | 说明 |
|------|------|
| 场模式 | DDF 有向距离 / 有符号 SDF |
| 场体素 (mm) | 0=自动 |
| 细阶段数据项 | 点-面（默认）/ DDF / SDF |
| 刚性预对齐、输出为新对象 | 同 SPARE |
| 执行 | `nonRigidRegisterSdf(doc, sourceBackendId, PluginPointCloudSdfParams, onFinished)` |

须与宿主同时升级到 **1.17.0+**（vtable）。算法见 PointCloudAlgorithm §3.6。

## 几何多分辨率金字塔（1.53.0+）

独立于单次 SPARE/SDF 调用：侧栏方法选 **几何金字塔**，源/目标须为网格。

| 控件 | 说明 |
|------|------|
| 基准边长 h (mm) | 0=源中位边长；L0=4h，其后对变形结果加密到 2h/h |
| 层求解器 | SDF（默认）或 SPARE |
| 刚性预对齐（仅粗层） | 仅 L0 交给求解器 ICP |
| 末层开启细阶段 | 默认开；10 万面级建议直接用单次 SPARE/SDF |
| 层求解器参数 | 选 SDF/SPARE 后复用对应「高级」区（采样半径、外迭代、对齐采样、平滑、ARAP） |
| 输出为新对象 | 默认开；拓扑为 **细层 remesh**，非原始源 |
| 执行 | `nonRigidRegisterPyramid(doc, sourceBackendId, PluginPointCloudPyramidParams, onFinished)` |

须宿主 **1.53.0+**。算法见 PointCloudAlgorithm §3.7。

## 网格后处理（1.9.0+，需 VcgAlgorithms.dll）

侧栏「网格后处理」区提供基于 vcglib 的网格操作：

| 操作 | 说明 | 参数 |
|------|------|------|
| **网格简化** | quadric-edge-collapse 面数精简 | 目标面数、质量阈值 |
| **Laplacian 平滑** | 快速拉普拉斯平滑（余切权重、边界保护） | 迭代次数 |
| **Taubin 平滑** | λ/μ 保形平滑 | 迭代次数、λ |
| **网格修复** | 去重/退化/重复面/非流形/可选填孔 | 填孔开关、孔洞最大边数 |
| **各向同性重网格** | 均匀三角形分布 | 目标边长(mm) |

典型流程：

1. 重建或导入网格 → 网格出现在「网格对象」下拉
2. 选中网格 → 显示面数/顶点数
3. 调整参数 → 点击操作按钮
4. 结果作为新网格对象创建，自动选中

菜单入口：**Tools → 点云 → 网格简化 / 网格平滑**

宿主需链接 `VcgAlgorithms.dll`；未链接时操作返回错误提示。

## 曲面重构（1.12.0+ 全流程，1.13.0+ 分阶段）

侧栏「曲面重构」区（与「网格后处理」共用 `m_meshTargetCombo`）：

| 控件 | 参数 / API |
|------|------------|
| 法矢光顺迭代 / 特征阈值 c0 | `normalSmoothIterations` / `featureThresholdC0` |
| 分块数（0=自动）/ 分块法向平滑 / 特征百分位 | `patchCountHint` / `partitionNormalSmoothIters` / `featureAnglePercentile` |
| 每边采样 n（间距=0 时） | `samplesPerPatchEdge` |
| UV 目标间距 / 最少·最多每边 | `targetUvSpacingMm`（0=固定 n）/ `minSamplesPerEdge` / `maxSamplesPerEdge`（0=无上限） |
| 拟合最多/边 / 拟合 UV 间距 | `maxFitGridPerEdge`（默认 9，0=不降采样上限）/ `fitUvSpacingMm`（0=仅用最多/边） |
| 采样率 k / 控制点密度 | `sampleRateFactor`（默认 2.0）/ `controlPointDensityFactor`（默认 0.5，AMRTO k_sample/k_type_gemodl） |
| NURBS 拟合模式 | `fitMode`（默认最小二乘+指定控制点数） |
| 混合带宽度（0=自动）| `blendStripWidth`（边界/交汇混合阶段） |
| 分块算法 | `partitionMode`（v3 / Hybrid / **CgalChartHybrid**） |
| 调和边界模式 | `harmonicBoundaryMode`（默认 GeodesicSquare，AMRTO 测地 square-border） |
| 光顺 ε / 最大迭代 | `fairingEpsilon` / `fairingMaxIterations` |
| 装配离散精度 (mm) | `tessellateLinearDeflectionMm`（装配阶段 B-rep 三角化精度） |
| **预处理后写入场景网格** | `exportPreprocessedMeshToScene`（临时 `Model`，名 `源名_预处理后`） |
| **预处理 → 装配输出**（8 按钮） | `beginMeshSurfaceReconstructSession` + `runMeshSurfaceReconstructStage` |
| **全流程** | `reconstructSurfaceFromMesh`（一次跑完，兼容 1.12） |
| **重置会话** | `clearMeshSurfaceReconstructSession` |

分阶段顺序（须按序执行，切换网格对象自动重置会话）：

1. **预处理** — Vcg 修复 + 法矢光顺；可选写入场景网格
2. **分块** — 四边域划分
3. **栅格采样** — PCA 物理等距栅格 + 曲面投影；质量未过时调和 UV（小 patch）或面心锚定回退（`pca-centroid`）
4. **NURBS 拟合** — AMRTO 式 centripetal 最小二乘（`NurbsSurfaceFitting`）；摘要含 `fitRejectApprox/Pole/FitGrid/FullGrid/MakeFace` 拒因统计
5. **边界混合** — 片间边界 C²
6. **交汇混合** — 多片交汇 C²
7. **光顺** — B 样条局部光顺
8. **装配输出** — Compound 装配 → 新 `BrepModel`

每阶段中文摘要写入：侧栏日志区、状态栏、RunInfo `[曲面重构]`（`PluginMeshSurfaceReconstructReport::stageSummaryZh`）。

典型调试流程：

1. Poisson 重建或导入 → 选中 `Model` 网格
2. 勾选「预处理后写入场景网格」→ 点 **预处理**，检查 `*_预处理后` 网格
3. 依次点 **分块** … **装配输出**，观察侧栏日志与 RunInfo
4. 或直接用 **全流程** 一次完成（菜单入口仍走全流程）

菜单：**Tools → 点云 → 曲面重构**（触发全流程）

详见 `docs/ARCHIVE_ZIP_LOCATION.txt`。

## 管状铸件算法（保留）

Plugin SDK / 侧栏 Tab / AI eature.build 已拆除。算法与 SelfTest 见 [GeometryAlgorithm/DEVELOPER_GUIDE.md](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.5（TubularGrinding.h）。

## 相关文档

- SDK：[`../CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md)
- 宿主：[`../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)
- 模板 B-rep 更新：`docs/ARCHIVE_ZIP_LOCATION.txt`
- VcgAlgorithms：[`../../Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md`](../../Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md)
