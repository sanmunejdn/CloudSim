# DESIGN — 网页端设备页桌面同步

```mermaid
flowchart TB
  RightDock --> modeBar[机器人_自定义设备]
  modeBar --> robotStack[指令_轴_轨迹_坐标系]
  modeBar --> deviceStack[设备指令_轴控制]
  deviceStack --> DeviceCommandPanel
  deviceStack --> JointAxesPanel
  LeftDock --> DevicesPanel[目录_组装]
  DevicesPanel -->|goDeviceCmd| DeviceCommandPanel
```

| 组件 | 职责 |
|------|------|
| `dockNavStore` | ws / deviceMode / deviceTab / robot tab |
| `deviceRuntimeStore` | selectedCustomDeviceId |
| `DeviceCommandPanel` | 姿态库 + DI→姿态绑定 |
| `JointAxesPanel` | 目标下拉 robot\|device |
| `CustomDevicesSection` | 左栏目录入口 |
