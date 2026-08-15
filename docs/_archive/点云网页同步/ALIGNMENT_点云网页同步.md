# ALIGNMENT — 点云网页同步

> 阶段：Align **已完成** → 见 `CONSENSUS_点云网页同步.md`  
> 依据：`Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md`、桌面 `PointCloudDockWidget`、现网 `GeometryOpsPanel` + stub `/api/pointcloud/op`  
> **范围决策（2026-08-09）**：尽量对齐桌面点云 Tab 全量；**特征构建**、**CAD 模板 B-rep 更新**暂不做。

## 1. 项目与任务特性

| 项 | 现状 |
|----|------|
| 桌面 | 侧栏双 Tab：**点云**（11 功能区）+ **特征构建**（管状分阶段）；经 `IPluginPointCloudHost` / Document / GeometryHost |
| Web UI | 右坞「点云」=`GeometryOpsPanel`：算法下拉 `boolean/downsample/normals/icp` +「执行算法作业」；**无参数表单、无列表、无日志** |
| Gateway | `POST /api/pointcloud/op`、`/api/geometry/op` 为 **queued stub + SSE**；`GET /api/pointcloud/chunk/:id` 空流；`POST /api/objects/import` 可真导入点云 |
| 视口 | `SceneViewport` **跳过** `geometryKind===1`（点云不渲染） |
| 历史规划 | `docs/_archive/网页端/TODO_网页端功能对等.md` P3「算法加厚」仍未做 |

## 2. 原始需求

同步桌面端点云页面功能到网页端（用户截图为当前 stub 页）。

## 3. 边界确认

### 明确纳入（已确认）

- 右坞「点云」按桌面分组重建 UI：
  - 文档与导入 / 点云对象列表 / 选中信息
  - 下采样 / 裁剪（含 **多边形拾取**）/ 预处理
  - 配准（ICP / SPARE / SDF·DDF）
  - 重建网格（Poisson / Scale-space）+ 导出 PLY
  - 网格后处理（简化 / 平滑 / 修复 / 重网格）
  - **曲面重构**（全流程 + 分阶段，对齐桌面 ≥1.12/1.13）
- Gateway 真接 Host `pointCloudHost()` 异步作业，结果写回 `objects[]` + SSE 进度
- 场景能显示点云：**混合渲染**（默认降采样 `THREE.Points` 预览；点数超过阈值走 chunk LOD）
- Web 视口 **多边形裁剪拾取**（对齐 `pickPolylineFromViewport`）

### 明确暂不做（已确认）

- **特征构建** Tab（`TubularGrindingDockWidget` / 管状分阶段）
- **CAD 模板 B-rep 更新**（含面拾取、粗/精匹配、面重构）

### 已决补充

- **boolean**：从点云页移除；几何算子另开入口（本次不实现）
- 降采样/chunk 阈值默认：**50 万点**（可在设置或常量调整，见 CONSENSUS）

### 明确不在本任务（除非另开）

- 改桌面插件业务算法本身
- PLC / AI / 一网页一进程部署

## 4. 需求理解

桌面「点云」不是单次 op 下拉，而是：

1. **文档与导入** → `importFileIntoActiveDocument(..., true)`
2. **点云对象列表 + 选中信息** → `queryPointCloudInfo` / `measurePointCloud`
3. **下采样 / 裁剪 / 预处理 / 配准 / 重建** → 各 `IPluginPointCloudHost::*` 异步回调
4. **网格后处理 / 曲面重构 / CAD 模板** → 网格与 B-rep 相关 Host API
5. **特征构建** → 独立 Tab + session API

Web 要对等，必须同时做：**UI 分组** + **真实 REST（参数/结果）** + **点云进场景**；只改 React 下拉无效。

## 5. 方案对比（架构）

| 方案 | 做法 | 优点 | 风险 |
|------|------|------|------|
| **A. Host 直连（推荐）** | WebGateway 调与插件相同的 `IPluginPointCloudHost` / Document API；React 按桌面分组做表单 | 与桌面单一真源、参数/结果一致 | 拾取需 Web 版 polyline/face pick |
| B. 绕过 Host 直链 PointCloudAlgorithm | Gateway 直接调算法 DLL | 少一层 | 重复插件会话/对象写回逻辑，易分叉 |
| C. 仅加厚现有 `/api/pointcloud/op` 字符串 op | 继续单 endpoint + op 名 | 改动面小 | 参数爆炸、难映射 SPARE/SDF/分阶段 |

**推荐 A**：按能力分 REST（或 `op` + 强类型 JSON params），UI 镜像桌面分区；分里程碑交付。

## 6. 建议里程碑（待共识后写入 CONSENSUS）

| 阶段 | 范围 | 验收要点 |
|------|------|----------|
| **M1 可见可管** | 导入 / 列表 / 信息；视口点云预览；真实 downsample + 预处理子集 | 选中有点数；场景能看见；作业后列表刷新 |
| **M2 裁剪+配准+重建** | box/sphere 裁剪；ICP；Poisson/Scale-space；导出 PLY | 与桌面同参可跑通 |
| **M3 网格后处理 + SPARE/SDF** | VCG 简化/平滑/修复/重网格；非刚性配准 | 依赖 Vcg / host 版本门控 |
| **M4 多边形拾取** | Web 视口折线绘制 → `cropPointCloudByPolyline` | 左键加点、右键/双击闭合、Esc 取消 |
| **M5 曲面重构** | 全流程 + 分阶段会话 + 侧栏日志 | 对齐桌面 stage API |
| **以后另开** | CAD 模板、特征构建 | 本次不做 |

## 7. 疑问澄清（按优先级）

1. ~~**首期范围**~~ → **已决**：桌面点云 Tab 全量对齐，排除特征构建与 CAD 模板。
2. ~~**点云渲染**~~ → **已决**：混合——默认降采样预览，超过阈值走 chunk LOD。
3. ~~**右坞信息架构**~~ → **已决**：单页纵向折叠分组，一个「点云」Tab，顺序对齐桌面 QGroupBox。
4. ~~**boolean**~~ → **已决**：从点云页移除；几何入口另开（本次不做）。

## 8. 参考路径

- 桌面指南：`src/Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md`
- 桌面 UI：`PointCloudDockWidget.cpp`、`TubularGrindingDockWidget.cpp`
- Web stub：`web/cloudsim-web-ui/src/docks/cloud/GeometryOpsPanel.tsx`
- 归档 TODO：`docs/_archive/网页端/TODO_网页端功能对等.md` §P3
