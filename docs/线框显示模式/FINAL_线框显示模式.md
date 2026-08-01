# FINAL：3D 视口线框显示模式

## 总结

视口「线框」对 BRep 改为经典 CAD 语义（隐藏填充 + 拓扑边），复用 Phase2 `edgePolylines`；Mesh 等仍用三角 `PolygonMode::LINE`。与导入 `showWireOutline` 解耦。

## 主要改动

| 文件 | 变更 |
|------|------|
| `BrepBackendVisual.h/.cpp` | 导出 `applyBrepViewportWireframe`；节点名 `brepViewportWireframe` |
| `OsgWidget.h/.cpp` | 重写 `setWireframeMode`；upsert/挂载后按模式应用 |
| `OsgWidgetColorController.cpp` | BRep 线框 Geode 着色压暗 |
| `BackendVisual` / `Widget` DEVELOPER_GUIDE | 文档同步 |

## 编译

- `BackendVisual`、`CloudSimHost`：**Debug\|x64** 与 **Release\|x64** 均成功

## 文档索引

- `ALIGNMENT` / `CONSENSUS` / `DESIGN` / `TASK` / `ACCEPTANCE` / `TODO`（本目录）
