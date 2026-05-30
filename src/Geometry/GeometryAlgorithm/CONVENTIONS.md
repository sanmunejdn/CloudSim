# GeometryAlgorithm conventions

> 模块说明与调用边界见 [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md)。

- 命名空间：`geoalgo`；导出宏 `GEOMETRY_ALGORITHM_API`（x64 DLL）。
- 单位：**mm**；三角 soup 为 `9*T` float，与 `MeshBackendData::triangleSoup` 一致。
- 枚举使用 `enum class`（`MeshDiscretizeMode`、`BrepBooleanOp` 等）。
- OCC `TopoDS_*` 仅出现在本模块公开头；SDK / 插件 / Data 对外用路径、soup、折线 DTO。
- 资源：OCC `Handle()` 走 RAII；禁止裸 `new`/`delete`。
- 注释：中文、简练，说明非显而易见的算法策略或单位约定。
