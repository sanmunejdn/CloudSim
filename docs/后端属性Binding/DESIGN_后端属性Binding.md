# DESIGN：后端属性 Binding

## 分层

| 层 | 职责 | 真源 |
|----|------|------|
| Domain | `worldMatrix` / `visible` / `color` / 派生成员 | C++ typed API |
| Binding | 面板/schema 唯一清单（descriptor + format/apply） | 静态表 + `extraPropertyBindings()` |
| Protocol | `snapshotPropertyRows` / `applyPropertyChange` | 基类统一实现 |
| Host | `semanticFlags` → VisualAspect → OSG flush | — |

## 行集合（按类型）

公共（有 pose 能力时）：`pose.frame`、`pose.x/y/z`、`rotation.x/y/z`；有 color 时：`color.r/g/b/a`。  
所有类型：`visible`、`core.id`/`core.name`/`core.class`。

| className | extra |
|-----------|-------|
| PointCloud / Brep / ParametricBrep | （无） |
| Model | 只读 `mesh.triangle_count` |
| Frame / CustomDevice | 可写 `axisLengthMm`（AffectsGeometry）；**无** color |

Follow 等仍走 `IBackendComponent`，不进 Binding 表。

## 非范围

- 不改 `IDataService` ABI、空间契约、OSG 同步结构
- 不动 `RobotInstruction` Attribute
- 不加插件运行时 Binding 注册表
- `PropertyBag` 保留（Follow `follow.targetName`）；不再影子同步 pose/color
