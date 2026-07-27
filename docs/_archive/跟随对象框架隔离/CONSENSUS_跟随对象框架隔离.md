# CONSENSUS — 跟随对象框架隔离

## 需求

工件/工具绑定跟随目标（如法兰）后：相对位姿恒定跟随；机器人装配不被 Follow 改写。

## 技术方案

| 角色 | 职责 |
|------|------|
| FK / `RobotSceneKinematics` | 唯一写 URDF 根与连杆世界位姿 |
| Follow 求解 | 仅更新非 URDF 自由物体 |

识别：`DocumentHost::isKinematicsOwnedBackend` ← `backendSourceType == "URDF"`。

防护点：

1. `applyHierarchyFollowBinding` — URDF child 直接 return，并卸已有 Follow。
2. `applyProjectEdgesFollowBindingAndSolve` — 跳过 URDF child；入口 strip。
3. `runBackendFollowSolveAndSync` — 入口 strip；sync 跳过 URDF。
4. `afterFollowPropertyEdited` — URDF 拒绑；绑定后仅脏集求解（取消 forced）。

## 验收标准

1. 对工件设 `follow.targetId=法兰`：工件不跳变，随后随法兰动。
2. 同上操作后机器人各连杆仍保持装配（不散架）。
3. 打开含 edges 的旧工程：连杆上的 hierarchy Follow 被卸掉，FK 恢复装配。
4. 非 URDF 自由物体之间的 Follow 链仍可用。
