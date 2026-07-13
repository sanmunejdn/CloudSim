# CAD 模板 + 扫描点云 → B-rep 面重构（worldMatrix v2）

点云插件「CAD 模板 B-rep 更新」：**反向配准**（固定扫描、变换 CAD 模板 `worldMatrix`）；用户在 3D 视图拖动 **CAD 工件** 与扫描（点云或 **Model 网格**）大致对齐后，后台世界系 soup ICP 精化 + 逐面几何调整，**注册新的 `BrepModel` 工件**（原模板保持不变）。

网格扫描：三角 soup 均匀采样为点+法向（最多 12 万点），ICP/面重构与点云共用 `registerScanToCadTemplate` / `updateBrepFromAlignedScan`（`sampleTriangleSoupToPointBuffers`）。

> **Breaking v2**：见 [`spatial_contract_world_pose.md`](spatial_contract_world_pose.md)。无 `alignedTemplateShape` / `alignedWorkXyz` cache；旧 JSON 需重导入。

## 1. 端到端流程

```text
PointCloudDockWidget
  ├─ [匹配] registerScanToCadTemplate
  │    ├─ prepareScanPointCloudForRegistration
  │    ├─ Job: geometry_backend_ops::registerScanToCadTemplate（世界系 ICP）
  │    ├─ applyTemplateRegistrationToVisual（template.worldMatrix += icpDeltaWorld）
  │    └─ TemplateBrepAlignCache（templateWorldMatrixAtRegister + icpRmse + report）
  │
  └─ [面重构] updateTemplateBrepFromAlignedScan
       ├─ Job: updateBrepFromAlignedScan（面归属 A）
       ├─ registerAdoptedBrepAndLoadScene
       └─ alignFaceUpdatedBrepWithTemplateVisual（new.worldMatrix = template.worldMatrix）
```

| 阶段 | 坐标系 |
|------|--------|
| 几何存储 | `geometry` = 文件/STEP 出生坐标，用户操作不改顶点 |
| 位姿 | 唯一权威 `worldMatrix`；`p_world = p_geometry × worldMatrix` |
| 配准 ICP | `scan_world`、`template_soup_world` 同世界系配对 → `icpDeltaWorld` |
| 配准写回 | `template.worldMatrix = icpDeltaWorld × template.worldMatrix` |
| 面更新输入 | `v_model = scan.geometry × scan.M × inv(template.M)` 与 **原始 STEP** 同系 |
| 新工件 | `new.geometry` = 调整后 STEP（文件系）；`new.worldMatrix` = `template.worldMatrix` |

## 2. 配准（世界系）

```text
scan_world     = scan.geometry × scan.worldMatrix
template_world = template.soup × template.worldMatrix
ICP → icpDeltaWorld
template.worldMatrix = icpDeltaWorld × template.worldMatrix（Host 预览同步 OSG）
geometry 两侧不变
```

**诊断**：`logRegistrationOverlapDiagnostic` 仅 `worldHits`（512 点采样）。

**门控**：

| 字段 | 含义 |
|------|------|
| `registrationPreviewOk` | 重叠预览通过 → `applyTemplateRegistrationToVisual` |
| `icpRmseGatePassed` | RMSE 门限通过 → 写入 cache 供面重构 |

预览失败：`restoreTemplateShapeFromStep` 恢复模板 **STEP 几何**（位姿保持 OSG 当前值）。

## 3. 面归属 A

实现：`GeometryBackendOps::updateBrepFromAlignedScan` → `scanPointsToTemplateModelFrame`：

```text
v_world = scan.geometry × scan.worldMatrix
v_model = v_world × inv(template.worldMatrix)
updateShapeFromPointCloud(originalSTEP, v_model, ...)
```

`faceBandMm` 可按 `icpRmseMm` 自适应放大。算法 DLL（`geoalgo::updateShapeFromPointCloud`）不持有 `worldMatrix`。

## 4. 新工件注册

1. `updateBrepFromAlignedScan` 写入 `result->brep`（文件系几何）
2. `registerAdoptedBrepAndLoadScene(..., resetViewToHome=false)`
3. `alignFaceUpdatedBrepWithTemplateVisual`：`setWorldMatrix(template.worldMatrix)` + `syncOuterPatFromBackend`
4. 模板、点云、新工件均保持可见

## 5. 关键文件

| 文件 | 职责 |
|------|------|
| `Data/GeometryBackendOps.cpp` | 世界系 ICP、面归属 A |
| `CloudSimPluginHost/PluginPointCloudHostImpl.cpp` | 插件编排、cache |
| `CloudSimPluginHost/DocumentPointCloudOps.cpp` | 预览写回、重叠诊断 |
| `GeometryAlgorithm/TemplateBrepUpdate.cpp` | 逐面拟合（输入已同系） |
