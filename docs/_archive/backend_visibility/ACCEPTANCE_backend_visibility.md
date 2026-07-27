# ACCEPTANCE：后端对象显示/隐藏

| 项 | 状态 | 说明 |
|----|------|------|
| Data `isVisible`/`setVisible` + JSON | 已完成 | `BackendDataBase` 公共字段 `visible` |
| `BackendObjectDto.visible` | 已完成 | `makeObjectSnapshot` 填充 |
| `IDataService::isVisible`/`setVisible` | 已完成 | Adapter + NullDataService |
| DocumentPage / Facade 双写 | 已完成 | 含插件旁路 facade |
| 树勾选读 DTO | 已完成 | `refreshOsgSceneTree` |
| 工程加载 apply NodeMask | 已完成 | 内嵌 + 文件回退路径 |
| VisualSync visible key | 已完成 | 属性变更同步 OSG |
| DEVELOPER_GUIDE | 已完成 | Data §3 / Widget §6.2 |
| 手工回归 | 待执行 | 见 TODO |

## 编译涉及工程

`Data` → `CloudSimCore` → `CloudSimHost` → `Widget`
