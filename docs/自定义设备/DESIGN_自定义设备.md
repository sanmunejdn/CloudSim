# DESIGN — 自定义设备

## 架构

```mermaid
flowchart TB
  Dev["CustomDeviceBackendData"]
  Child["Model/Brep 子件"]
  Dev -->|setParent + Follow| Child
  AxisUI["RobotAxisControlWidget 目标切换"] -->|q| Apply["CustomDeviceKinematics"]
  Apply -->|W0*T(q)| Dev
  Apply --> FollowSolve["FollowSolve"]
  FollowSolve --> Child
```

## 分层

| 层 | 职责 |
|----|------|
| CloudSimCore | `kClassCustomDevice` / catalog / helpers |
| Data | `CustomDeviceBackendData` + 轴配置结构 + JSON |
| BackendVisual | `CustomDeviceBackendVisual` |
| Host | 注册加载、工程 IO 门禁、挂接 Follow |
| RobotScene | `CustomDeviceKinematics`：转 `RobotExternal` 并写位姿 |
| RobotWidget | 向导、轴控目标切换 |

## 接口

- `CustomDeviceBackendData::axes()` / `setAxes` / `qValues` / `setQValues` / `baseWorldW0` / `setBaseWorldW0`
- `CustomDeviceKinematics::applyQ(device, mgr, poseSink)`：漂移 unbake → 合成 → `setWorldMatrix` + poseSink
- `attachChildToCustomDevice(host, deviceId, childId)`：setParent + OSG parent + Follow
- 新建向导组件列表：`从场景选择`（已加载 Mesh/Brep）与 `导入文件` 均可挂为多子件；`移除` 时 detach + 清 hierarchy Follow
- `RobotAxisControlWidget` / Controller：目标枚举含自定义设备 id

## 异常

- 缺设备/子件：跳过并日志
- `q` 钳位到 `[lower, upper]`
- 工程无 `geometry.kind=customDevice` 的空路径对象：按 `isCustomDeviceClassName` 放行（同 Frame）
