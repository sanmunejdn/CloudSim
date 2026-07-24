# DESIGN — 碰撞检测

## 架构

```mermaid
flowchart LR
  Mesh[MeshBackendData] --> Sync[BackendCollisionSync]
  Brep[BrepBackendData] --> Disc[discretizeShapeHandleToMesh]
  Disc --> Sync
  Sync --> World[CollisionWorld]
  UI[Dock 开关] --> Settings[RobotCollisionSettings]
  Settings --> Plan[planMotionOnHost]
  Plan --> World
```

## 模块

| 模块 | 职责 |
|------|------|
| CollisionAlgorithm | AABB 宽相 + 三角窄相；可选 coal |
| RobotCollisionSettings | 文档级 enabled / securityMarginMm |
| BackendCollisionSync | 后端几何同步、ACM、轨迹抽样 |
| RobotCollisionSettingsWidget | 仿真 Dock 页签 |

## 开关语义

- 默认 `enabled=false`：plan 不调用碰撞
- `enabled=true`：抽样轨迹 FK → checkAll，命中则 plan 失败
