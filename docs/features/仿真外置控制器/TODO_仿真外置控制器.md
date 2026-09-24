# TODO — 仿真外置控制器（P0–P3 后）

## 已完成

| 波次 | 内容 |
|------|------|
| P0 | 真实 actualJointRad、无 STEP 空转、断连保持、dtMs 警告、P0_语义附录 |
| P1 | `CloudSimControllerSample.exe`、`resource/controllers/`、对话框可跑 C++/Python、打开目录 |
| P2 | Headless OsgWidget 头卫统一；Headless/Gateway Debug+Release 可编 |
| P3 | `P3_立项说明.md` + `STEP_POSE` 预留 ERROR |

## 后续（按需）

| 项 | 说明 |
|----|------|
| STEP_POSE + Host IK | 接 `RobotTeachIk`，升 protocolVer=2 |
| 多会话 / ROS / SimReal 门面 | 见 P3 立项说明 |
| Gateway 手工点验 | `POST /api/robot/external-controller/start` + Python STEP |
