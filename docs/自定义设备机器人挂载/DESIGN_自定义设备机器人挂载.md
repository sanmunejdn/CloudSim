# DESIGN — 自定义设备机器人法兰挂载

## 数据

`CustomDeviceRobotMountComponent` on `CustomDeviceBackendData` → `project.json` `components[]`（元数据：法兰 id、安装 Frame、`frameInDeviceW0`、`T_tool`）。

运行时耦合由设备根 `FollowAttachment` 表达：

```text
W_device = W_flange × T_local
T_local  = T_tool × inv(T_frame_in_device)   // 挂载时 bake
```

## 运动学

- `applyLinkJointGraph` 以当前 `device.worldMatrix()` 为 W0 展开 Link/Joint，**不**在 `CustomDeviceKinematics` 内重算挂载位姿。
- 机器人 FK → `markFollowDirty(连杆)` → `runFollowSolveAndSync` → `refreshCustomDevicesFollowingKinematicsTargets`（对已 Follow 运动学目标的自定义设备 `applyQ`）。

## Phase 1 约束

安装 Frame 须在设备根或 `fixed=true` 的 Link 下。

## Host

| API | 说明 |
|-----|------|
| `mountCustomDeviceToFlange` | 配置设备根 Follow + 元数据 |
| `unmountCustomDeviceFromRobot` | 清除 Follow |
| `refreshCustomDevicesFollowingKinematicsTargets` | FK + Follow 后刷新 |
| `rebakeMountedCustomDevicesFollowLocals` | 工具系变更后重算 T_local |
| `mountCustomDeviceToRobotFlange` | Web/JSON 入口 |

## UI

- 桌面：`CustomDeviceAssemblyDialog` 挂载区 + `ICustomDeviceAssemblyHost`
- Web：`CustomDeviceAssemblyDialog.tsx` + Gateway 路由

## 文件

- `Data/inc/CustomDeviceRobotMountComponent.h`
- `Host/.../CustomDeviceRobotMountOps.cpp`
- `RobotScene/source/CustomDeviceKinematics.cpp`
