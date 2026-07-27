# DESIGN — 转换工件型轨迹算子

```mermaid
flowchart LR
  Session[TrajectoryEditSession]
  Cap[tryCaptureCurrentRobotTcpPose]
  Engine[TrajectoryPipelineEngine]
  Ctx[TrajectoryOpExecutionContext]
  Op[ToWorkpieceInHandOp]
  Session --> Cap
  Cap -->|"setWorkpieceReferenceInBase"| Engine
  Engine -->|"applyGeometryOp"| Ctx
  Ctx --> Op
  Op --> Unified[UnifiedTrajectory]
```

## 模块

| 模块 | 职责 |
|------|------|
| `ToWorkpieceInHandParams` | 外部 TCP + 速度开关 |
| `TrajectoryOpExecutionContext` | 注入 `workpieceReferenceInBase` |
| `TrajectoryPipelineEngine` | 缓存参考位姿并填入 ctx |
| `TrajectoryEditSession` | 执行前捕获当前 TCP |
| `ToWorkpieceInHandOp` | validate / processPath |

## 核心变换

对 scope 内点：`B_T_Eout = B_T_TCP * inv(inv(B_T_Wf) * B_T_Ei)`，即 `B_T_TCP * inv(B_T_Ei) * B_T_Wf`（\(F_T_W=I\)）。
