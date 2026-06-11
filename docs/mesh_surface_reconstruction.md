# 网格曲面重构（BrepModel）

将三角网格重构为分片 B 样条 B-rep，输出新的 `BrepModel` 后端，**不覆盖**源网格。

## 管线

```
三角 soup (mm)
  → Vcg 修复 + 法矢光顺 (Ch2, Data 层)
  → 四边域分块 + UV 分箱栅格采样 (Ch3.2)
  → 初始 B 样条片 / 平面回退 (GeomAPI_PointsToBSplineSurface)
  → 边界/交汇 C² 混合 (Ch3.3, 首版简化)
  → B 样条局部光顺 (Ch4, 首版简化)
  → Compound 装配 + 输出校验 → ShapeHandle → BrepBackendData
```

## 装配与校验

| 步骤 | 实现 | 说明 |
|------|------|------|
| 装配 | `MeshSurfaceReconstructionAssemble.cpp` | 开放曲面用 `TopoDS_Compound`，不做 Sewing |
| 三角化门禁 | `MeshSurfaceReconstructionValidate.cpp` | 单面三角数 ≤ 8000，禁止空 mesh |
| 包围盒门禁 | 同上 | 输出对角线 / 输入对角线 ≤ 3.0 |

拟合失败或极点/face 校验未过的分片自动回退为平面四边形。

## 手工验收

1. 点云 → Poisson Auto 重建，得到 `Model` 网格
2. 侧栏「网格后处理」区选中该网格
3. 「曲面重构」区默认参数 → **重构曲面**
4. 树中出现新 `BrepModel`，自动选中；原网格 id 与面数不变
5. 视口可拾取 B-rep 面/边；导出 STEP 可在外部 CAD 打开
6. RunInfo 输出 patch 数、max deviation、光顺指标；失败时有明确错误（如包围盒无效、三角化过密）

## API 链路

| 层 | 符号 |
|----|------|
| UI | `PointCloudDockWidget` → `IPluginPointCloudHost::reconstructSurfaceFromMesh` |
| Host | `runMeshToBrepJob` → `registerAdoptedBrepAndLoadScene(skipRebase=false)` → `inheritBrepVisualPoseFromSourceMesh`（内层质心与源网格对齐） |
| Data | `geometry_backend_ops::reconstructBrepFromMeshSoup` |
| Geo | `geoalgo::reconstructBrepFromMeshSoup` |
| Vcg | `vcgalgo::smoothMeshByNormalAdjustment` |

## 自检

```cpp
std::string err;
const bool ok = geoalgo::runSelfTest(&err);  // 含 meshSurfaceReconstruct
```

## 已知限制（首版）

- 分块默认最多 6 片；大片网格建议先简化再重构
- C² 混合与光顺为简化实现，复杂模型可能以平面片为主
- 质量与分块均衡性仍在迭代中

## 相关文档

- [`GeometryAlgorithm/DEVELOPER_GUIDE.md`](../src/Geometry/GeometryAlgorithm/DEVELOPER_GUIDE.md) §3.4
- [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../src/Plugins/PointCloudPlugin/DEVELOPER_GUIDE.md) §曲面重构
