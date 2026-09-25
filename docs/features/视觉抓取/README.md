# 视觉抓取（MVP）

工业相机侧栏第三 Tab：用手工/场景物体位姿 + 手眼结果合成接近 → 抓取 → 抬起 TCP，经 `IPluginRobotHost::planAndConfirmTcpWaypoints` 复用仿真 Dock「碰撞与规划」的 `planToTcpPose` 链路，预览确认后写入活动程序。

## 坐标系

| 位姿源 | 含义 |
|--------|------|
| 手工基座系 | 自旋框直接当机器人基座系 TCP |
| 手工相机系 | `T_base_obj = T_base_cam * T_cam_obj`（Eye-to-hand：`T_best`；Eye-in-hand：当前法兰 × `T_flange_cam`） |
| 场景选中 | 对象世界位姿变换到活动机器人基座系后填入自旋框 |

抓取偏移：接近 = 抓取 Z + `approachZMm`，抬起 = 抓取 Z + `retractZMm`；可选锁定欧拉为当前 TCP。

## 与碰撞规划页关系

- **不**在抓取 Tab 再造规划器下拉；`planningSpace` / `plannerId` / 碰撞开关 / 规划时限取自碰撞页当前 `settings()`
- 按段调用 `robot_path::planToTcpPose`，含画面复验；成功后 OSG 预览，弹窗确认后 `emitRawTrajectoryToProgram`（与碰撞页「确认」同源）
- 取消写入时预览保留，仍可到碰撞页点确认

## 宿主 API（1.55.0+）

`IPluginHostContext::robotHost()` → `IPluginRobotHost`：

- `getActiveRobotTcpPose`
- `getSelectedBackendWorldPose`（已变到基座系）
- `planAndConfirmTcpWaypoints`

## 工程持久化

根 JSON 键 `industrialCameraVisionGrasp`：接近/抬起高度与是否锁欧拉。

## 限制（本波不做）

实时检测、CAD 配准、夹爪 IO、外置传图、一键 Run、网页对等。
