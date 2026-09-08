# ACCEPTANCE — RobotPathPlanning

## 编译

- [x] `RobotPathPlanning.vcxproj` Debug|x64 → `bin/x64d/RobotPathPlanning.dll`
- [x] `RobotPathPlanning.vcxproj` Release|x64 → `bin/x64/RobotPathPlanning.dll`
- [x] `RobotWidget.vcxproj` Debug|x64 / Release|x64（含 ProjectReference 与链接）

## API / 算法

- [x] `planToTcpPose`：空请求 / 缺 world 返回明确 `errMsg`
- [x] 成功路径：`jointTrajectoryRad` 与 `tcpPoses` 等长
- [x] 状态有效性：URDF 限位 + FK 绑定位姿（`fk-bind`）+ `CollisionWorld::checkAll`
- [x] 边离散：`longestValidSegmentRad` 线段碰撞校验；densify 后复检
- [x] 位姿目标：`flangeFromToolOrigin` + `solveArmPoseDampedLeastSquares`
- [x] OMPL 级联：BIT* → InformedRRT* → RRT* → RRTConnect（无 OMPL 时内置 RRT）
- [x] UI 画面闸门：`validateJointTrajectory`

## UI

- [x] Dock「碰撞与规划」：黑白名单、起终点、规划/清除/确认插入
- [x] `BackendCollisionSync::rebuildWorld` → `planToTcpPose` → OSG 预览 → `insertRawTrajectoryBetween`
- [x] 不调用 PathPlan / 不写程序指令（仅插入 Raw/Pmid）

## 文档

- [x] `DEVELOPER_GUIDE.md`（BIT*、位姿公式、非职责）
- [x] `MODULE_DEVELOPER_GUIDES.md` 索引条目
- [x] `bin/SDK/ompl/README.md`（OMPL 构建与 `CLOUDSIM_HAS_OMPL`）

## 待运行时人工抽检

- [ ] 真实 URDF + 障碍：规划 → 确认插入 → 回放无 Pmid 碰撞
- [ ] 障碍阻挡时规划失败并显示可读 `errMsg`
- [ ] 画面复验失败时不插入 Pmid
