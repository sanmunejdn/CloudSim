# PointCloudPlugin 示例

点云处理插件，演示 **1.2.0+** SDK：`IPluginPointCloudHost` + 侧栏 UI。

## 构建与部署

| 项 | 说明 |
|----|------|
| 工程 | `PointCloudPlugin.vcxproj`（x64，v142，Qt 5.14.2） |
| 链接 | **仅** `CloudSimPluginSDK.lib` |
| 部署 | `bin/x64(d)/plugins/com.cloudsim.pointcloud/plugin.json` + `PointCloudPlugin.dll` |
| `minHostVersion` | `"1.12.0"`（曲面重构） |

## 运行时

- 侧栏 Tab **点云** / **Point Cloud**：导入、列表、下采样、裁剪（包围盒/球/多边形）、预处理、ICP、重建
- 菜单 **Tools → Point Cloud**（中文下子菜单标题为 **点云**）
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

1. 导入 STEP → `BrepModel`、扫描 PLY → 点云
2. 3D 视图手动对齐点云与 CAD
3. 选择模板 B-rep；可选 **选择面…**（`geometryHost()->pickStepElementFromViewport`）累积面索引，空列表=全部面
4. **匹配 (ICP)** → `registerScanToCadTemplate`
5. **面重构** → `updateTemplateBrepFromAlignedScan`（须先匹配）；详见 [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)

## 网格后处理（1.9.0+，需 VcgAlgorithms.dll）

侧栏「网格后处理」区提供基于 vcglib 的网格操作：

| 操作 | 说明 | 参数 |
|------|------|------|
| **网格简化** | quadric-edge-collapse 面数精简 | 目标面数、质量阈值 |
| **Laplacian 平滑** | 快速拉普拉斯平滑 | 迭代次数 |
| **隐式平滑** | Implicit Fairing 保形平滑 | 迭代次数 |
| **网格修复** | 去退化面/重复顶点/非流形 | 自动 |
| **各向同性重网格** | 均匀三角形分布 | 目标边长(mm) |

典型流程：

1. 重建或导入网格 → 网格出现在「网格对象」下拉
2. 选中网格 → 显示面数/顶点数
3. 调整参数 → 点击操作按钮
4. 结果作为新网格对象创建，自动选中

菜单入口：**Tools → 点云 → 网格简化 / 网格平滑**

宿主需链接 `VcgAlgorithms.dll`；未链接时操作返回错误提示。

## 曲面重构（1.12.0+）

侧栏「曲面重构」区（与「网格后处理」共用 `m_meshTargetCombo`）：

| 控件 | 参数 |
|------|------|
| 法矢光顺迭代 / 特征阈值 c0 | `normalSmoothIterations` / `featureThresholdC0` |
| 分块数（0=自动）/ 每边采样 n | `patchCountHint` / `samplesPerPatchEdge` |
| 光顺 ε / 最大迭代 | `fairingEpsilon` / `fairingMaxIterations` |
| **重构曲面** | `reconstructSurfaceFromMesh` |

典型流程：

1. Poisson 重建或导入 → 选中 `Model` 网格
2. 默认参数 → **重构曲面**
3. 生成新 **`BrepModel`** 后端（源网格保留）；树中自动选中
4. 摘要显示分块数、最大偏差、光顺指标

菜单：**Tools → 点云 → 曲面重构**

详见 [`docs/mesh_surface_reconstruction.md`](../../docs/mesh_surface_reconstruction.md)。

## 相关文档

- SDK：[`../CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md)
- 宿主：[`../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)
- 模板 B-rep 更新：[`../../docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)
- VcgAlgorithms：[`../../Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md`](../../Geometry/VcgAlgorithms/DEVELOPER_GUIDE.md)
