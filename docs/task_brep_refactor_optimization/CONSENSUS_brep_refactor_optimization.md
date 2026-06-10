# B-rep 重构逻辑优化 - 共识文档

## 1. 明确需求描述

### 1.1 核心变更

| 维度 | 当前方式 | 目标方式 |
|------|----------|----------|
| **配准方向** | 变换扫描点云匹配模板 | 变换模板B-rep匹配扫描点云 |
| **面更新策略** | 从点集重新拟合新曲面 | 用扫描点作为控制点，直接修改原始B-rep几何定义 |
| **面类型覆盖** | Plane、Cylinder、Freeform | 完整支持：Plane、Cylinder、Cone、Sphere、Toroid、BSpline、Other |

### 1.2 详细需求

#### 需求1：反向配准

**目标**：ICP配准时变换模板B-rep，而非扫描点云

**实现要点**：
- 修改`alignScanToTemplateRegistration`函数
- ICP的目标/源角色互换
- 输出变换应用于模板shape而非扫描点云
- 保持与现有坐标系约定的兼容性

#### 需求2：特征类型驱动的B-rep调整

**目标**：根据原始CAD面类型，用扫描点直接修改原始B-rep几何定义

**各面类型处理策略**：

| 面类型 | 调整策略 |
|--------|----------|
| **Plane** | 用扫描点拟合约束，调整平面原点和法向量 |
| **Cylinder** | 用扫描点约束，调整轴线位置/方向和半径 |
| **Cone** | 用扫描点约束，调整轴线、半角和顶点 |
| **Sphere** | 用扫描点约束，调整球心和半径 |
| **Toroid** | 用扫描点约束，调整主半径、次半径和轴线 |
| **BSpline** | 用扫描点作为控制点，调整B样条曲面的控制点网格 |
| **Other** | 降级为当前的拟合方式 |

#### 需求3：点面归属链接

**目标**：建立扫描点与CAD面的精确映射关系

**实现要点**：
- 保持现有的`faceBandMm` + 法线门控机制
- 输出每个面的归属点集
- 用于后续的特征类型驱动调整

## 2. 技术实现方案

### 2.1 反向配准方案

**修改文件**：`Data/source/GeometryBackendOps.cpp`

**关键修改点**：

```cpp
// 当前：变换扫描点云
applyIsometryInPlace(workXyz, icpStep);

// 目标：变换模板shape
TopoDS_Shape transformedTemplate = BRepBuilderAPI_Transform(templateNative, trsf).Shape();
```

**注意事项**：
- ICP算法本身不需要修改（`RegistrationRigid.cpp`）
- 只需在编排层反转变换的应用对象
- 需要更新`scanToTemplate`的语义（变为`templateToScan`）

### 2.2 特征类型驱动调整方案

**修改文件**：`Geometry/GeometryAlgorithm/source/TemplateBrepUpdate.cpp`

**新增函数**：

```cpp
// 根据面类型调整原始B-rep几何
bool adjustFaceGeometry(
    const TopoDS_Face& originalFace,
    const std::vector<Vec3>& assignedPoints,
    TopoDS_Face& adjustedFace,
    FaceUpdateAction& action);

// 各面类型的具体调整实现
bool adjustPlaneFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out);
bool adjustCylinderFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out);
bool adjustConeFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out);
bool adjustSphereFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out);
bool adjustToroidFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out);
bool adjustBSplineFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out);
```

**Plane调整算法**：
1. 用扫描点拟合最小二乘平面
2. 计算相对于原始平面的平移和旋转
3. 应用变换到原始平面的`Geom_Plane`

**Cylinder调整算法**：
1. 用扫描点拟合轴线和半径
2. 计算相对于原始圆柱的轴线偏移和半径差
3. 应用调整到原始`Geom_CylindricalSurface`

**BSpline调整算法**：
1. 将扫描点投影到原始B样条曲面的UV参数域
2. 用投影点作为约束，优化控制点
3. 使用`Geom_BSplineSurface::SetPole`逐点调整

### 2.3 面类型扩展方案

**新增FaceUpdateAction枚举值**：

```cpp
enum class FaceUpdateAction
{
    Unchanged,
    PlaneRefit,      // 保留，向后兼容
    CylinderRefit,   // 保留，向后兼容
    FreeformRefit,   // 保留，向后兼容
    SkippedNoPoints,
    // 新增
    ConeRefit,
    SphereRefit,
    ToroidRefit,
    PlaneAdjusted,
    CylinderAdjusted,
    ConeAdjusted,
    SphereAdjusted,
    ToroidAdjusted,
    BSplineAdjusted
};
```

## 3. 任务边界限制

### 3.1 在范围内

1. 修改`TemplateBrepUpdate.cpp`的面更新逻辑
2. 修改`GeometryBackendOps.cpp`的配准编排
3. 扩展`FaceUpdateAction`枚举
4. 实现各面类型的调整算法

### 3.2 在范围外

1. 修改ICP/RANSAC核心算法
2. 修改UI层显示
3. 修改点云预处理流程

## 4. 验收标准

### 4.1 功能验收

- [ ] 反向配准：模板B-rep能够被ICP变换以匹配扫描点云
- [ ] 特征类型识别：能够正确识别所有OCCT面类型
- [ ] 面调整：非样条曲面能够用扫描点调整原始几何定义
- [ ] BSpline调整：B样条曲面能够用扫描点调整控制点
- [ ] 拓扑保持：调整后的B-rep拓扑结构保持一致
- [ ] 质量门控：`globalMaxDeviationMm`和`qualityPassed`正确计算

### 4.2 性能验收

- [ ] 面调整时间不超过原拟合时间的2倍
- [ ] 内存使用无显著增长

### 4.3 兼容性验收

- [ ] 现有`TemplateBrepUpdateParams`接口不变
- [ ] 现有`TemplateBrepUpdateResult`接口不变
- [ ] 插件SDK接口兼容

## 5. 确认清单

- [x] 需求边界清晰无歧义
- [x] 技术方案与现有架构对齐
- [x] 验收标准具体可测试
- [x] 所有关键假设已确认

## 6. 关键假设

1. OCCT的`BRepBuilderAPI_Transform`能够正确变换B-rep几何
2. `GeomAPI_ProjectPointOnSurf`能够将点投影到B样条曲面的UV域
3. `Geom_BSplineSurface::SetPole`能够在保持连续性的情况下调整控制点
