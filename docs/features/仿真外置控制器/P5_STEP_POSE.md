# P5 — STEP_POSE + Host IK

> protocolVer=**2**；v1 关节 STEP 仍可用。

## STEP_POSE

```json
{
  "type": "STEP_POSE",
  "protocolVer": 2,
  "dtMs": 16,
  "targetTcpMm": [x, y, z],
  "targetEulerDeg": [rx, ry, rz],
  "frame": "robot_base"
}
```

Host：`RobotTeachIk::solveTeachIk` → pending 关节 → 与 STEP 相同 `tickApply` / `STEP_REPLY`。

| code | 含义 |
|------|------|
| PROTOCOL_MISMATCH | STEP_POSE 未带 ver=2 |
| BAD_FRAME | frame 非 robot_base |
| BAD_DT | dtMs 非法 |
| IK_FAILED | 无解 |
| OUT_OF_LIMITS | 解越限 |

HELLO（ver=2）ACK 含 `supportsStepPose: true`。

## 客户端

- Python：`RobotController.step_pose`；样例 `example_step_pose.py`
- C++：`IControllerClient::stepPose`

## 不做

`frame=world`、外轴联立优化、真机。
