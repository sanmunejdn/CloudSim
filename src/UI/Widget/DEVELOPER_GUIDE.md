
### 4.1b DocumentPage 机器人元数据收口（2026）

`DocumentPage` 实现以下新接口（通过 `DocumentHost` 注入）：

| 接口 | 来源 | 说明 |
|------|------|------|
| `IPerLinkKinematicsHost` | `CloudSimHost` | per-link 机器人运动学计算委托 |
| `IPerLinkRobotStateAccessor` | `CloudSimHost` | 状态快照提取与结果应用 |

**调用路径**：
- `MainWindow` / `RobotWidget` 通过 `doc->robot()` 或 `doc->render()` 触发
- 实际计算由 `PerLinkKinematicsHostImpl`（Host 编译单元）执行
- `DocumentPage` 只负责 DTO 转换与内部状态同步

**收口效果**：`MainWindow.cpp` 已移除 `OsgWidget.h`；`DocumentPage.cpp` 仍保留 Robot* 头用于访问器实现中的类型转换（未来可通过 Pimpl 进一步隔离）。