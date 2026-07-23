# CONSENSUS：三点圆弧指令（CIRC / ARC）

## 需求与验收

| 项 | 约定 |
|----|------|
| 类型 | `Type::ARC`，JSON `"arc"` |
| 数据 | End：`pose`/`eulerDeg`；Via：`viaPose`/`viaEulerDeg` + `context.viaTransform*` |
| 起点 | 规划时由 `context.currentJointRadCsv` FK 得到（链式种子） |
| 示教 | 两步：Via → End；取消：切换机器人/清空/显式取消 |
| 规划 | `ArcPlanner`：三点定圆 → 弦长采样 → 逐点 URDF IK；共线失败 |
| 预览 | 选中摆到 End（与 LINE 同） |
| Run | `jointTrajectoryRad` 插帧 |
| 导出 | Canonical 透传 `arc`+via；品牌脚本本期不改 |

## 技术约束

- 几何：`RobotKinematics`（`CircularArcGeometry`）
- 规划/模型：`RobotScene`
- UI/编排：`RobotWidget`
- 指纹含 via，避免缓存脏读
- `isMotionWaypointType` 含 ARC

## 验收标准

见计划 §7：示教 JSON、圆弧轨迹、共线错误、预览/Run、指纹、PTP/LINE 回归、x64 编译。
