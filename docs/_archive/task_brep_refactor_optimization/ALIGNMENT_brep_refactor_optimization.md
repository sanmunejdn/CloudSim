# B-rep 重构逻辑优化 - 对齐文档

## 1. 原始需求

用户希望优化当前的CAD特征重构逻辑：

1. **反向匹配**：粗匹配和精匹配阶段反过来，将后端对象（模板B-rep）匹配到扫描点云上
2. **链接点所属面**：建立扫描点与CAD面的映射关系
3. **特征类型驱动重构**：根据原始CAD特征类型选择不同的重构方式
   - 非样条曲面（平面、圆柱等）：将该面所属点作为控制点，对面进行调整，然后替换
   - 样条曲面：保持现有拟合方式
   - **核心目标**：不是全部拟合，而是根据特征类型选择不同策略

## 2. 现有系统分析

### 2.1 当前流程（模板→扫描方向）

```
用户手动对齐（OSG世界坐标）
  ↓
transformScanPointsToTemplateModelFrame（扫描→STEP坐标）
  ↓
registerScanToCadTemplate
  ├─ preparePointCloudWork（体素/离群/法线）
  ├─ sampleShapeSurfacePoints（模板面网格采样）
  └─ alignScanToTemplateRegistration
       ├─ bbox中心对齐（非预对齐）
       ├─ PCA朝向（非预对齐）
       ├─ [可选RANSAC]（非预对齐）
       ├─ 粗ICP（点-点）
       └─ 精ICP（点-面）
  ↓
updateBrepFromAlignedScan
  └─ updateShapeFromPointCloud
       ├─ 点→面归属（faceBandMm + 法线门控）
       ├─ 逐面拟合（PlaneRefit/CylinderRefit/FreeformRefit）
       └─ ShapeFix修复拓扑
```

### 2.2 关键代码位置

| 模块 | 文件 | 职责 |
|------|------|------|
| 配准编排 | `Data/source/GeometryBackendOps.cpp` | ICP流水线、Job入口 |
| 面更新 | `Geometry/GeometryAlgorithm/source/TemplateBrepUpdate.cpp` | 点面归属、拟合逻辑 |
| 点云算法 | `Geometry/PointCloudAlgorithm/source/RegistrationRigid.cpp` | ICP实现 |
| 全局配准 | `Geometry/PointCloudAlgorithm/source/RegistrationGlobal.cpp` | RANSAC粗配准 |

### 2.3 当前面更新逻辑（`TemplateBrepUpdate.cpp:330-402`）

```cpp
// refitFaceFromPoints 当前逻辑：
if (surfType == GeomAbs_Plane) {
    // 从点集拟合新平面
    fitPlaneFromPoints(assignedPts, pln);
    makePlanarFaceWithWire(pln, wire, outFace);
    outAction = FaceUpdateAction::PlaneRefit;
}
else if (surfType == GeomAbs_Cylinder) {
    // 从点集拟合新圆柱
    fitCylinderFromPoints(assignedPts, axis, radius);
    // 创建新圆柱面
    outAction = FaceUpdateAction::CylinderRefit;
}
else {
    // B样条曲面拟合（自由曲面）
    makeFreeformFaceFromPoints(assignedPts, wire, outFace);
    outAction = FaceUpdateAction::FreeformRefit;
}
```

**问题**：当前所有面都是从点集"重新拟合"，而不是"调整"原始CAD面。

## 3. 需求理解

### 3.1 反向匹配的含义

用户希望将**模板B-rep对象**匹配到**扫描点云**上，即：
- **当前**：扫描点云 → 模板（ICP变换扫描点云）
- **期望**：模板 → 扫描点云（ICP变换模板）

这在数学上是等价的（逆变换），但在实现上有差异：
- 变换对象不同
- 误差度量方向不同

### 3.2 特征类型驱动重构的含义

用户希望根据原始CAD面的类型，选择不同的"调整"策略：

| 面类型 | 当前方式 | 期望方式 |
|--------|----------|----------|
| 平面（Plane） | 从点集拟合新平面 | 用点集作为控制点调整原平面（平移/旋转） |
| 圆柱（Cylinder） | 从点集拟合新圆柱 | 用点集调整原圆柱（轴线/半径微调） |
| 圆锥（Cone） | 未处理 | 用点集调整原圆锥参数 |
| 球面（Sphere） | 未处理 | 用点集调整原球面参数 |
| B样条曲面（BSpline） | 从点集拟合新曲面 | 保持拟合方式，或用点集调整控制点 |

### 3.3 "链接点所属面"的含义

建立扫描点与CAD面的精确映射：
- 每个扫描点标记它属于哪个CAD面
- 用于后续的特征类型驱动重构

## 4. 边界确认

### 4.1 在范围内

1. 修改`TemplateBrepUpdate.cpp`中的`refitFaceFromPoints`函数
2. 添加新的面类型处理（Cone、Sphere等）
3. 实现"调整"而非"拟合"的逻辑
4. 可选：修改配准方向

### 4.2 在范围外

1. 修改ICP算法本身（`RegistrationRigid.cpp`）
2. 修改RANSAC算法（`RegistrationGlobal.cpp`）
3. 修改UI层（`PointCloudDockWidget`）

## 5. 疑问澄清

### 5.1 关于反向匹配

**问题**：用户说"将后端对象匹配到扫描点云上"，是指：
- A) 变换模板B-rep以匹配扫描点云（变换模板）
- B) 保持现有方向，但调整配准参数

**推测**：可能是A，因为用户说"反过来"。

### 5.2 关于"调整"而非"拟合"

**问题**：对于非样条曲面，"将该面所属点作为控制点，对面进行调整"具体是指：
- A) 保持原始曲面类型，只调整参数（如平面法向、圆柱半径）
- B) 用点集作为约束，优化原始曲面的控制点

**推测**：对于解析曲面（Plane/Cylinder/Cone/Sphere），应该是A；对于B样条曲面，应该是B。

### 5.3 关于面类型覆盖

**问题**：需要处理哪些面类型？
- 当前：Plane、Cylinder、Freeform（BSpline）
- 可能需要：Cone、Sphere、Toroid、Other

## 6. 项目特性规范

- **语言**：C++17
- **几何内核**：OCCT 7.9（OpenCASCADE）
- **点云库**：CGAL 5.5.2
- **矩阵库**：Eigen
- **坐标系**：STEP文件坐标（mm）
- **变换语义**：列向量 `p' = T * p`
