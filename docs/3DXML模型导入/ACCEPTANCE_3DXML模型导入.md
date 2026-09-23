# ACCEPTANCE：3DXML 模型导入

| 项 | 结果 |
|----|------|
| `MeshBackendData_3dxml.cpp` 解析 ZIP+Manifest+ProductStructure+3DRep | 已实现 |
| 三角化 strips/triangles/fans + RelativeMatrix 烘焙 | 已实现 |
| `load3dxmlHierarchyFromFile` / `backend_io` `.3dxml` / Host 层级导入 | 已接线 |
| 文件过滤器含 `*.3dxml` | Widget / RobotWidget / WebGateway / AI 选择器 |
| Debug\|x64 Data + CloudSimHost | 通过 |
| Release\|x64 Data + CloudSimHost | 通过 |

## 导入扩展

几何导入走 Host `IGeometryFileImporter::parse` → `ImportParseResult` → `applyGeometryImportParse`；后缀由 `GeometryFileImporterRegistry` 懒注册。新增格式 = 新 Importer + `registerBuiltinGeometryImporters` 内 `add`。

## 手工点验

用 CATIA / SolidWorks 导出的 `.3dxml` 经「导入模型」打开，场景树应出现装配父节点及各 part 网格。
