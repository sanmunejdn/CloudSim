# TrajectoryAlgorithm 模块开发文档

## 1. 模块定位

`TrajectoryAlgorithm` 是轨迹管道的 **框架库**：接口、Registry、Codec、参数解析与变换数学；具体原子块在 [`TrajectoryAlgorithmBuiltins`](../TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md)。

## 2. 执行上下文注入

[`TrajectoryOpExecutionContext`](inc/TrajectoryOpExecutionContext.h) 由 `TrajectoryPipelineEngine::applyGeometryOp` 填充：

| 字段 | 来源 | 用途 |
|------|------|------|
| `program` | Engine | scope 解析 |
| `geometryProjection` | RobotScene 适配器 | ProjectToGeometry |
| `nonRigidTrajectoryWarp` | RobotScene 适配器 | NonRigidRegistration |
| `workpieceReferenceInBase` | Session 捕获当前 TCP 后 Engine setter | ToWorkpieceInHand |
| `reachabilityProbe` | Session 注入 TeachIk 服务 | ReachabilityFilter |

Builtins **不得**依赖 RobotWidget；机器人状态只经 Context / Engine 注入。

## 3. 相关文档

- Builtins：[`../TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md`](../TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE.md)
- 管道引擎：[`../RobotScene/DEVELOPER_GUIDE.md`](../RobotScene/DEVELOPER_GUIDE.md)
