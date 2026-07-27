# DESIGN — 非刚性配准轨迹算子

```mermaid
flowchart LR
  Op[NonRigidRegistrationOp]
  Ctx[TrajectoryOpExecutionContext]
  Adapter[RobotSceneNonRigidTrajectoryWarp]
  Resolver[resolveTrajectoryGeometry]
  Spare[nonRigidRegister Spare APIs]
  Op --> Ctx --> Adapter --> Resolver
  Adapter --> Spare
```

## 模块

| 模块 | 职责 |
|------|------|
| `INonRigidTrajectoryWarp` | TrajectoryAlgorithm 抽象 |
| `TrajectoryNonRigidWarp.cpp` | 绑定 + SPARE 分发 + 写回 |
| `NonRigidRegistrationOp` | validate / processPath |
