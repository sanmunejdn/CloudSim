# CONSENSUS：3DXML 模型导入

## 需求

- 支持 CATIA / SolidWorks 导出的 `.3dxml`（ZIP + Manifest + ProductStructure + `.3DRep`）导入
- 解析逻辑对齐 `robdts-dev` 的 `dxmlStructureRead` / `dxmlRead` / `HPLFactoryWorkpiece3Dxml` 三角化

## 方案

| 项 | 约定 |
|----|------|
| 输出 | `MeshHierarchyPart` 列表（与 DXF 层级导入同一路径） |
| 装配 | 每个 Product 子节点 → 一个 part；顶点经 RelativeMatrix（12 元）烘焙到装配系 |
| 单件 | 无 children 时仍输出 parts（一个或多个 3DRep） |
| ZIP | zlib inflate（STORE + DEFLATE）；OCCT 自带 zlib |
| XML | 内嵌 `Data/third_party/tinyxml2` |
| 导入路由 | `GeometryFileImporterRegistry` → `DxmlGeometryImporter::parse` → `applyGeometryImportParse` |
| UI | 桌面/网页过滤器由 `geometryOpenModelFileFilter` 从注册表生成（含 `*.3dxml`） |

## 验收

- Debug|x64 与 Release|x64 `Data` → `CloudSimHost`（及使用导入滤镜的 Widget / WebGateway）编译通过
- 有效 3dxml 可产生非空 triangleSoup 并注册到场景树
