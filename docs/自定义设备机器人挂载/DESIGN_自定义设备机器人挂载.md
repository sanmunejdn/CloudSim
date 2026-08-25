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
- 挂载预同步 `applyQ` 使用 `refreshRestFromGeometry=false`，避免未启用挂载时误刷新 `parentToChildRest`。
- 解除挂载后 `syncCustomDeviceKinematicsAfterRootPoseChange` 传播连杆与旋转中心 Frame 视觉。

### 机器人 FK 后刷新链

```text
notifyRobotKinematicsAppliedToScene
  → runBackendFollowSolveAndSync
       Follow（跨部件；挂载设备根跳过写世界）
       → 未挂载 follower：compound 或设备 applyQ
       → 受限 Follow（跟 compound 动过的 target）
       → refreshCustomDevicesFollowingKinematicsTargets
            updateMountedDeviceWorldFromRobotTcp + applyQ（连杆 FK + compound）
       → 再受限 Follow（跟挂载连杆子 Solid 的工件等）
       → flushVisualSync
```

- 通用 Follow 求解器**跳过**已挂载 `CustomDevice` 根的世界写入（由 `updateMountedDeviceWorldFromRobotTcp` 独占）。
- 同部件 STEP 子件：**无** hierarchy Follow；由 `backend_compound` / `applyToSink` 刚体更新。**禁止**在 `flushCustomDeviceLinkGeometryVisual` 内再调 `runFollowSolveAndSync`。
- 工具系变更：`rebakeMountedCustomDevicesFollowLocals` 重算 `T_local`。
- 概念细则：`docs/Follow与Compound分流/CONSENSUS_Follow与Compound分流.md`。

## Phase 1 约束

安装 Frame 须在设备根或 `fixed=true` 的 Link 下。

## Host

| API | 说明 |
|-----|------|
| `mountCustomDeviceToFlange` | 配置设备根 Follow + 元数据；预 FK / flush / 安装坐标系解析 |
| `unmountCustomDeviceFromRobot` | 清除 Follow + `syncCustomDeviceKinematicsAfterRootPoseChange` |
| `updateMountedDeviceWorldFromRobotTcp` | TCP 对齐设备根与安装 Frame，并 `applyQ` |
| `refreshCustomDevicesFollowingKinematicsTargets` | 机器人 FK 后刷新已挂载设备 |
| `rebakeMountedCustomDevicesFollowLocals` | 工具系变更后重算 T_local |
| `mountCustomDeviceToRobotFlange` | Web/JSON：预 FK、`captureTcpPose`、挂载前后 notify |
| `syncCustomDeviceKinematicsAfterRootPoseChange` | 根位姿变更后 applyQ + 视觉 flush |
| `ensureCustomDeviceLinkKinematicsOwnership` | 轴控准备：登记连杆独占；不刷新 rest |
| `finalizeCustomDeviceLinkJointGraph` | **仅组装提交后**刷新 rest；禁止姿态库热路径 |
| `flushCustomDeviceLinkGeometryVisual` | 连杆 + compound 下挂子件 OSG（位姿在 `applyToSink`） |
| `flushCustomDeviceMotionCenterFrameVisual` | 旋转中心 Frame OSG 同步 |

## UI

- 桌面：`CustomDeviceAssemblyDialog` 挂载区 + `ICustomDeviceAssemblyHost`（预 FK + notify）
- Web：`CustomDeviceAssemblyDialog.tsx` + Gateway；`mount` body 可选 `jointAnglesRad`

## 文件

- `Data/inc/CustomDeviceRobotMountComponent.h`
- `Host/.../CustomDeviceRobotMountOps.cpp`
- `Host/.../CustomDeviceHostOps.cpp`
- `RobotScene/source/CustomDeviceKinematics.cpp`
