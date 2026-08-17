# CloudSimMeshTrajectorySDK 开发指南

## 定位

`CloudSimMeshTrajectorySDK.dll` 提供 **无 Qt 依赖** 的 mesh 轨迹会话：三角 soup 快照、区域选择缓冲、参数 POD、调用 `geoalgo::generateMeshTrajectory`。

视口交互、实时预览与 PathPlan 挂接由宿主 `RobotWidget`（`MeshTrajectoryPageWidget`）经 `IRobotOsgViewHost` / `IRobotMainWindowHost` 完成。

## 消费者

- [`TrajectoryGenerationPageWidget`](../../UI/RobotWidget/inc/TrajectoryGenerationPageWidget.h) — Dock **轨迹生成** 子页 **Mesh**
- [`MeshTrajectoryPageWidget`](../../UI/RobotWidget/inc/MeshTrajectoryPageWidget.h) — 方法/参数 UI 与预览同步

## 头文件

| 头文件 | 说明 |
|--------|------|
| `MeshTrajectorySession.h` | 会话：选择、`generateRawPath`、`specJsonUtf8` |
| `MeshTrajectoryTypes.h` | `MeshTrajectorySelectionMode` 等 POD |
| `mesh_trajectory_sdk_global.h` | 导出宏 |

## 依赖

| 模块 | 用途 |
|------|------|
| `GeometryAlgorithm.dll` | `MeshTrajectory.h` — 截面求交、B 样条拟合（§算法） |
| `RobotScene.dll` | `MeshTrajectoryIngress.h` — `RawPath` → `RawTrajectory` |
| `RobotWidget` | 预览、`TrajectoryEditSession::setRawTrajectory` |

## 端到端数据流

```text
MeshBackendData.triangleSoup
  → MeshTrajectorySession::beginMesh(backendId, soup)
  → [B 样条] 视口三角面选择 → applyTriangleSelection
  → UI 写入 MeshTrajectorySpec（method / crossSection / bspline / discretize）
  → generateRawPath → geoalgo::RawPath（模型系，可含 segmentEndExclusive）
  → importMeshRawPathToRawTrajectory
  → TrajectoryEditSession::setRawTrajectory
  → applyMeshLocalRawTrajectoryPreviewToOsg（世界系叠加层）
```

**坐标**：算法与 `RawTrajectory.points` 均在 **mesh 模型系 mm**；预览/程序经 `FeaturePickTransform` 乘 `getBackendRootWorldMatrix`（`resolvePickScopeBackendId` 后）。

---

## 两种轨迹方法

### 总览

| | 截面法 | B 样条曲面拟合 |
|---|--------|----------------|
| `MeshTrajectoryMethod` | `CrossSection` | `BsplineRegion` |
| 必需输入 | 平面原点 + 法向 | ≥3 选中三角面 |
| 区域选择 UI | 隐藏（会话内选区仍可能残留，见 §注意） | 点选 / 刷选 / 套索 |
| 离散参数 | `discretize.stepMm`、切向/法向开关 | `uvCountU/V`、`gridAngleDeg`、`fitUvSpacingMm`、`traceMode` |
| 3D 预览 | 半透明截面片 + 可选编辑罗盘 | 选中三角高亮 + 拟合曲面网格 |
| 算法文档 | [`GeometryAlgorithm` §3.1a](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) | 同上 |

### 截面法逻辑

```text
1. intersectPlaneWithTriangleSoup
     平面 × 每个三角 → 0~1 线段；可选 triFilter 限制在 region.triangleIndices
2. chainSegmentsToPolylines
     端点 snap(0.05mm) 拼折线 → vector<MeshTrajectoryPolyline>
3. discretizeAllMeshTrajectoryPolylines
     按折线弧长降序；每条 discretizeMeshTrajectoryPolyline(stepMm)
     依次 append 到 RawPath.points，记录 segmentEndExclusive
```

**输出语义**：一条平面可与网格有多条不交交线（例如股骨多圈/多弧）；**全部**保留为多段轨迹。`RawTrajectory.segmentEndExclusive` 与 `RawPath` 同构；预览时每段独立 `LINE_STRIP`；`emitRawTrajectoryToProgram` 多段时建多个 PathPlan 输出分组（`*_S1`、`*_S2`…）。

**UI 预览（`MeshTrajectoryPageWidget`）**

| 控件 / API | 行为 |
|------------|------|
| 「显示截面」复选框 | 控制 `showMeshSectionPlane` / `setMeshSectionPlanePreviewVisible` |
| Origin/Normal spinbox | 实时 `syncSectionPlanePreview` |
| 「相机法向」 | `getCameraViewDirectionInBackendModel` |
| 「编辑截面」 | `beginMeshSectionPlaneEdit` 启用罗盘拖拽；关闭后仅保留截面片 |
| 截面渲染 | 平面片挂 `backendObjectsGroup`、参与深度测试；罗盘单独无光照 StateSet |

### B 样条区域逻辑

```text
1. buildRegionFrame — 选中三角质心 + 平均法向 → 区域切平面 (U,V)
2. gridAngleDeg 旋转扫描方向 rotU/rotV
3. nu×nv 格点：projectGridPointToRegion（UV 平面点 → 最近选中三角面投影）
4. `NurbsSurfaceFitting::fitNurbsSurfaceFromGrid`（centripetal 最小二乘，与曲面重构同源）→ `Handle(Geom_BSplineSurface)`
5. 曲面参数域均匀 outU×outV 采样位姿（可选切向/法向）
6. appendTraceModePoints — 蛇形 U / 蛇形 V / 栅格点序
```

**与截面法的本质区别**：轨迹在 **拟合曲面** 上采样，**不是** 平面与 mesh 求交。绕曲/重叠选区时单张 B 样条可能只贴合一侧；预览曲面（`buildBsplineRegionSurfacePreview`）范围可大于实际采样折线。

**UI 预览**

| 触发 | 行为 |
|------|------|
| 选区变化 / U/V / spacing / 角度 | `syncBsplineSurfacePreview`（≥3 三角） |
| 切换方法 | 清除截面或拟合 overlay |

---

## 会话 API（`MeshTrajectorySession`）

| 方法 | 说明 |
|------|------|
| `beginMesh` | 绑定 backendId + soup 快照；清空 undo 栈 |
| `applyTriangleSelection` / `clearSelection` / `invertSelection` | 维护 `m_selectedTriangles`；同步到 `spec.region` |
| `generateRawPath` | 合并 `m_spec` + 当前选区 → `generateMeshTrajectory` |
| `specJsonUtf8` | 持久化到 `RawTrajectory.sourceFeatureJson` |

### 注意：截面法与选区

`generateRawPath` **始终**写入 `spec.region.triangleIndices = m_selectedTriangles`。截面法 UI 虽隐藏选区面板，若会话中仍有选中三角，求交会被 **过滤** 到这些面上。截面法建议 **清除选择**（0/N）后再生成，否则交线可能在选区边界断开。

---

## 视口 Host API（摘要）

截面（`IRobotOsgViewHost` → `OsgWidget`）：

| API | 说明 |
|-----|------|
| `showMeshSectionPlane` | 持久显示截面片（无法向编辑回调） |
| `beginMeshSectionPlaneEdit` / `updateMeshSectionPlanePose` / `endMeshSectionPlaneEdit` | 罗盘编辑 + spinbox 回调 |
| `hideMeshSectionPlane` / `setMeshSectionPlanePreviewVisible` | 移除或隐藏 overlay |
| `getCameraViewDirectionInBackendModel` | 相机法向按钮 |

B 样条 / 选区：

| API | 说明 |
|-----|------|
| `setMeshTrianglePickTool` | Click / Brush / Polyline |
| `showMeshTriangleHighlight` / `clearMeshTriangleHighlight` | 选中三角高亮 |
| `showMeshFittedSurfacePreview` / `clearMeshFittedSurfacePreview` | 拟合曲面半透明网格 |

轨迹预览：

| API | 说明 |
|-----|------|
| `setRawTrajectoryOverlay(points, segmentEndExclusive)` | 多段折线 + 点；段间不连线 |
| `applyMeshLocalRawTrajectoryPreviewToOsg` | 模型系 raw → 世界 overlay |

退出阶段：`MeshTrajectoryPageWidget` 析构 **不** 访问 OSG；清理由 `OsgWidget` 析构或切换方法/host 时完成。

---

## JSON（`MeshTrajectorySpec`）

```json
{
  "schemaVersion": 1,
  "method": "CrossSection",
  "workpiece": { "backendIdUtf8": "mesh_01", "frameId": "workpiece" },
  "region": { "triangleIndices": [] },
  "crossSection": {
    "planeOriginMm": [0, 0, 0],
    "planeNormal": [0, 0, 1]
  },
  "discretize": { "stepMm": 2.0, "outputTangent": true, "outputNormal": true },
  "bspline": {
    "uvCountU": 16, "uvCountV": 16,
    "gridAngleDeg": 0.0, "fitUvSpacingMm": 0.0,
    "traceMode": "USerpentine"
  }
}
```

`method`: `"CrossSection"` | `"BsplineRegion"`。

---

## 相关文档

| 文档 | 内容 |
|------|------|
| [`GeometryAlgorithm/DEVELOPER_GUIDE.md`](../../Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.1a | 算法实现与 API |
| [`RobotWidget/DEVELOPER_GUIDE.md`](../../UI/RobotWidget/DEVELOPER_GUIDE.md) §Mesh 轨迹生成 | UI 页签与绑定 |
| [`OsgWidgetCore/DEVELOPER_GUIDE.md`](../../UI/OsgWidgetCore/DEVELOPER_GUIDE.md) §5.8b | 截面/拟合 overlay 渲染 |
| [`RobotScene/DEVELOPER_GUIDE.md`](../../Robot/RobotScene/DEVELOPER_GUIDE.md) | `RawTrajectory`、`emitRawTrajectoryToProgram`、PathPlan |
