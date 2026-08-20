# CONSENSUS — RobotPathPlanning

## 验收标准

1. `RobotPathPlanning.dll` 输出至 `bin/x64d`、`bin/x64`；Debug|x64 与 Release|x64 编译通过。
2. `planToTcpPose`：成功时 `jointTrajectoryRad` 与 `tcpPoses` 等长且 ≥2；失败返回 `errMsg`。
3. 场景 mesh 障碍下路径经边离散 + `CollisionWorld::checkAll` 不穿模。
4. 仿真 Dock「规划到目标 / 清除预览」可见 OSG 折线。
5. `DEVELOPER_GUIDE.md` 与 MoveIt 概念对照完整。
