# ACCEPTANCE — UI 思想落地 P0–P2

## P0

| 项 | 状态 | 说明 |
|----|------|------|
| T0.1 Job cancel | 完成 | `JobSystem::enqueueCancellable`/`cancel`；SDK `enqueueCancellableJob`/`cancelJob`；ABI `0x00013400` |
| T0.2 onDocumentClosed | 完成 | `closeDocumentTab` → `PluginManager::invokeDocumentClosed`（Host 导出，避免 Widget 链接未导出的 `PluginHostContext`）；三插件 destroyOnClose |
| T0.3 指令树 id | 完成 | `kInstrIdRole`；`findSharedById`；去掉裸指针身份 |
| T0.4 Web 增量 | 完成 | `sceneStore` merge；Gateway 队列 256 + stop disconnect |

## P1

| 项 | 状态 | 说明 |
|----|------|------|
| T1.1 objectName | 完成 | Dock 不用 title；侧栏不再写指针 hex |
| T1.2 可行轴缓存 | 完成 | 并入 `PlanResultCache` |
| T1.3 tokens | 完成 | `ApplicationStyle::tokens`；AssemblyMate 用 danger token；shell.css CSS 变量 |
| T1.4 桌面树 | 完成 | 结构 rebuild 后不再全量 `applyCurrentDocumentVisibilityToScene` |

## P2

| 项 | 状态 | 说明 |
|----|------|------|
| T2.1 桌面 Units Model | 完成 | `BackendUnitsTreeBinder` 绑到 `QTreeView` + `QStandardItemModel`（`BackendUnitsDisplayForest` DTO 仍为数据源） |
| T2.2 Web UnitsTree | 完成 | ≥200 对象时根/子节点限量渲染 |
| 指令程序树 Model | 未做 | 拖放/逻辑分支仍依赖 `QTreeWidgetItem`，见 TODO |

## 编译（2026-08-19）

Debug\|x64 与 Release\|x64 均通过：

- `Widget.vcxproj`
- `ProcessFlowPlugin` / `EngineeringDrawingPlugin` / `GeometricModelingPlugin`
- `CloudSimWebGateway.vcxproj`（补 `GeometryAlgorithm/inc` 以解析 `ShapeHandle.h`）
- `CloudSimWeb.vcxproj`（postbuild 将 Vite 产物落到 `bin\x64d\web` / `bin\x64\web`）
