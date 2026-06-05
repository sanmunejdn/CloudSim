# GeometryAlgorithm 模块开发文档

编码约定见 [`CONVENTIONS.md`](CONVENTIONS.md)。

## 1. 模块定位

`GeometryAlgorithm` 是 **CAD/网格几何算法 DLL**：OCC 点/线/面离散、网格离散、求交、B-rep 布尔、线/面融合；CGAL 网格布尔。不依赖 Qt/OSG。

| 属性 | 说明 |
|------|------|
| 输出 | x64 `GeometryAlgorithm.dll` → `bin/x64(d)/` |
| 命名空间 | `geoalgo` |
| 依赖 | OCCT 7.9、CGAL 5.5.2、Boost、Eigen |

## 2. 数据契约

| 类型 | 布局 |
|------|------|
| 折线 | `3*N` float（有序顶点，mm） |
| 三角 soup | `9*T` float（每三角 3 顶点 xyz） |
| STEP 路径 | 本地窄字节（`QFile::encodeName`） |

## 3. API 总览

| 头文件 | 功能 |
|--------|------|
| `Discretize.h` | Edge/Wire/Shape 折线与三角 soup；STEP 单件/层级 |
| `BrepImportArtifacts.h` | BREP 导入预处理：显示 soup、面/边离散、按 `ShapeHandle` 共享缓存 |
| `MeshDiscretize.h` | 自适应/UV 网格/线管带网格；质量预设 |
| `ShapeIo.h` / `ShapeQuery.h` | STEP 读入；按索引求交/离散编排 |
| `Intersection.h` | 线线、线面、面面、形体截面 |
| `BrepBoolean.h` | OCC Fuse/Common/Cut → Shape 或 mesh |
| `WireOps.h` / `ShellOps.h` | 线融合、面缝合 |
| `GeoMeshBoolean.h` | CGAL 三角网格布尔 |
| `FeatureSpec.h` | **CAD 轨迹特征**：`FeatureSpec` / `RawPath` / `FeatureCatalog`；统一离散入口 `discretizeFeature` |
| `SelfTest.h` | `runSelfTest` |

### 网格离散模式（`MeshDiscretizeMode`）

- `AdaptiveTriangulation`：默认 STEP 路径（`BRepMesh_IncrementalMesh`）
- `UniformRelative` / `UVStructuredGrid`：质量与 UV 结构化
- `WireTubeMesh` / `WireRibbonMesh`：折线扫掠
- `RemeshSoup` / `PointCloudSurface`：预留（当前构建返回未实现）

## 3.1 CAD 轨迹特征离散（`FeatureSpec.h`）

**设计原则**：对机器人执行层只有轨迹；工艺（焊缝/涂胶/打磨）不在本 DLL 区分，仅通过下游 `RawTrajectory` 编辑配方体现。

| API | 职责 |
|-----|------|
| `validateFeatureSpec` / `validateFeatureSpecWithShape` | 结构校验；后者加载 STEP 检查边/面索引 |
| `discretizeFeature` | **唯一离散入口**；按 `FeatureKind` 内部分派 |
| `discretizeFeatures` | 批量离散 |
| `enumerateFeatureCatalog` | 边/面拓扑摘要 + 启发式标签（焊缝/涂胶/打磨候选） |
| `featureSpecFromJson` / `featureSpecToJson` | UI / AI / 配置文件共用 JSON |
| `suggestFeaturesFromCatalog` | 规则回退：按意图从目录生成 `FeatureSpec[]` |

| `FeatureKind` | 说明 |
|---------------|------|
| `EdgeChain` | 单边或有序边链 |
| `FaceBoundary` | 面外轮廓 |
| `FaceIntersection` | 两面交线 |
| `FaceOffsetCurve` | 面内边沿法向偏置 |
| `FaceUVGrid` | 面内 UV 扫描点族（打磨栅格） |
| `Composite` | 子特征顺序拼接 |
| `SyntheticPolyline` | 外部/LLM 点列透传（无 STEP 降级路径） |

实现：`source/FeatureDiscretize.cpp`；复用 `Discretize` / `Intersection` / `ShapeQuery` / `WireOps`，不对外暴露按类型的顶层 API。

**JSON 示例**（与计划文档一致）：

```json
{
  "schemaVersion": 1,
  "featureId": "seam_01",
  "workpiece": { "backendIdUtf8": "mesh_12", "stepPathUtf8": "D:/part.step" },
  "kind": "FaceIntersection",
  "refs": { "faceIndices": [3, 7] },
  "discretize": { "stepMm": 2.0, "outputTangent": true, "outputNormal": true }
}
```

### 3.2 BREP 导入预处理（`BrepImportArtifacts.h`）

| API | 说明 |
|-----|------|
| `buildBrepImportArtifacts(shape, out)` | 一次 `tessellateShapePerFaceMedium` + 边离散 + 面-边索引；供显示/拾取/线框共用 |
| `getOrBuildBrepImportArtifacts(shape)` | 按 `ShapeHandle::isSame()` 缓存；装配多零件共享同一 assembly shape 时零重复离散 |
| `clearBrepImportArtifactsCache()` | 测试或工程切换时清空 |

| `BrepImportArtifacts` 字段 | 用途 |
|---------------------------|------|
| `displaySoup` / `triangleFaceIndex` | 三角显示与面 id 映射 |
| `faceSoups` | 每面局部 soup（BREP 面拾取/高亮） |
| `edgePolylines` / `faceEdgeIndices` | 边线框与面-边拓扑 |

**层级拓扑（无 tessellation）**：`collectShapeHierarchyTopology(shape, outParts)` 仅遍历 OCCT 装配树，输出 `MeshHierarchyPart` 路径/显示名；**不**填充 `triangleSoup`。Data 层 `BrepBackendData::loadStepHierarchyFromFile` 将其转为 `BrepHierarchyPart` 并为各零件共享同一 `assembly` `ShapeHandle`。

带 tessellation 的 `collectShapeHierarchy` 仍供网格路径（DXF/旧 STEP mesh 回退）使用。

## 4. Data 薄包装

[`GeometryBackendOps.h`](../Data/inc/GeometryBackendOps.h)（`geometry_backend_ops`）转发 STEP 路径级 API，供 `CloudSimPluginHost` 调用。STEP 导入仍经 `MeshBackendData::loadFromFile` → `geoalgo::tessellateStepFile`。

特征轨迹 API 另见 [`GeometryRef.h`](../Data/inc/GeometryRef.h)：`resolveGeometryRef`、`discretizeFeature`、`enumerateFeatureCatalog` 等（`geometry_backend_ops` 命名空间）。

## 5. 插件 SDK（1.5.0+）

- `IPluginGeometryHost`：异步离散/求交/布尔（见 `CloudSimPluginSDK/inc/IPluginGeometryHost.h`）
- 宿主：`PluginGeometryHostImpl` → `DocumentGeometryOps` → `geometry_backend_ops`
- 示例：`plugins/com.cloudsim.geometry/GeometryPlugin`

## 6. 自检

```cpp
std::string err;
const bool ok = geoalgo::runSelfTest(&err);
```

## 7. 相关文档

- [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../../Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md)
- [`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)
- [`RobotScene/DEVELOPER_GUIDE.md`](../../Robot/RobotScene/DEVELOPER_GUIDE.md) §14 — `RawTrajectory` 编辑流水线
