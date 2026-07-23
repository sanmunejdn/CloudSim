# ACCEPTANCE：三点圆弧指令（CIRC / ARC）

| # | 验收项 | 状态 | 说明 |
|---|--------|------|------|
| 1 | `Type::ARC` + JSON `"arc"` + via/end 字段 | 通过（代码） | `ArcInstruction`、Factory、schema |
| 2 | `isMotionWaypointType` 含 ARC | 通过（代码） | collect / 树 / 属性面板自动纳入 |
| 3 | 两步示教 Via→End | 通过（代码） | Controller pending + UI「确认终点」 |
| 4 | `CircularArcGeometry` 三点定圆/采样 | 通过（代码） | `RobotKinematics` |
| 5 | `ArcPlanner` + Executor | 通过（代码） | 笛卡尔弧采样+IK；共线失败 |
| 6 | 指纹含 via | 通过（代码） | `computePlanFingerprint` |
| 7 | 预览摆 End / Run 走轨迹 | 通过（代码） | 与 LINE 同路径 |
| 8 | Canonical 透传 viaTcp | 通过（代码） | 品牌脚本本期不改 |
| 9 | PTP/LINE 回归 | 待手工 | 添加/预览/Run 不变 |
| 10 | x64 编译 | 部分 | `RobotKinematics` Debug x64 **通过**；`RobotScene`/`RobotWidget` 源文件已无 error C 编译，链接缺本机 `CloudSimCore.lib`/`BackendVisual.lib`（环境依赖，非本改动） |

## 手工建议

1. 示教：PTP/LINE 到 Start → ARC 点一次（Via）→ 移到 End 再点 → 树出现 ARC
2. Run：TCP 应呈圆弧；共线三点应规划失败并提示
3. 改属性面板 Via 坐标后重 Run，轨迹应变化（fingerprint 失效）
