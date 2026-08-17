# 主程序（活跃入口）

内建工作区：`modeId` 为空字符串，顶栏显示「主程序 / Main」。三维场景、工程文档、机器人仿真与轨迹、侧栏工具插件均在此模式下使用。

## 代码入口

| 区域 | 文档 |
|------|------|
| 可执行入口 | [`CloudSim/DEVELOPER_GUIDE.md`](../../src/App/CloudSim/DEVELOPER_GUIDE.md) |
| 主窗口 / Units / 多文档 | [`Widget/DEVELOPER_GUIDE.md`](../../src/UI/Widget/DEVELOPER_GUIDE.md) |
| 文档宿主 / 开保存 | [`CloudSimHost/DEVELOPER_GUIDE.md`](../../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) |
| 仿真 / 设备 / 轨迹 UI | [`RobotWidget/DEVELOPER_GUIDE.md`](../../src/UI/RobotWidget/DEVELOPER_GUIDE.md) |
| 指令与规划 | [`RobotScene/DEVELOPER_GUIDE.md`](../../src/Robot/RobotScene/DEVELOPER_GUIDE.md) |
| 后端对象 | [`Data/DEVELOPER_GUIDE.md`](../../src/Data/Data/DEVELOPER_GUIDE.md)、[`后端对象与软件模式/`](../后端对象与软件模式/) |
| 世界坐标契约 | [`spatial_contract_world_pose.md`](../spatial_contract_world_pose.md) |
| 网页端 | [`cloudsim-web-ui`](../../web/cloudsim-web-ui/DEVELOPER_GUIDE.md)、[`CloudSimWebGateway`](../../src/Web/CloudSimWebGateway/DEVELOPER_GUIDE.md) |

模块总表见 [`MODULE_DEVELOPER_GUIDES.md`](../MODULE_DEVELOPER_GUIDES.md)。模式切换机制见 [`_archive/WorkspaceModeSwitcher/`](../_archive/WorkspaceModeSwitcher/)。

## 与其它模式

切到几何建模 / 工艺流程 / 工程图时由对应插件 `claimWorkspaceMode`；返回主程序调用宿主 `returnToMainWorkspace`（空 `modeId`）。
