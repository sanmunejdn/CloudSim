# 工艺流程（活跃入口）

独立 SDK 插件 `com.cloudsim.processflow`：工艺流程图编辑 + DES 离散事件仿真（节拍、缓冲、调度、瓶颈）。与 RobotWidget「轨迹工艺预设」无关。

## 开发入口

| 文档 | 说明 |
|------|------|
| [ProcessFlowPlugin/README.md](../../src/Plugins/ProcessFlowPlugin/README.md) | 功能、DES、JobSet、AI 桥接、构建部署（**日常真源**） |
| [CloudSimPluginSDK/DEVELOPER_GUIDE.md](../../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) | 插件 ABI；Host 工艺 AI 桥接 |
| [后端对象与软件模式/](../后端对象与软件模式/) | 侧车键 `processFlow` |

## 历史归档

| 路径 | 说明 |
|------|------|
| [_archive/工艺流程仿真插件/](../_archive/工艺流程仿真插件/) | UI / DES 设计、验收、TODO |
| [_archive/工艺流程_一期_体验闭环/](../_archive/工艺流程_一期_体验闭环/) | 一期 |
| [_archive/工艺流程_二期_工业加深/](../_archive/工艺流程_二期_工业加深/) | 二期 |
| [_archive/工艺流程_三期_执行智能/](../_archive/工艺流程_三期_执行智能/) | 三期 |
| [_archive/工艺流程AI助手/](../_archive/工艺流程AI助手/) | process.flow 域 |
| [_archive/WorkspaceModeSwitcher/](../_archive/WorkspaceModeSwitcher/) | 顶栏模式切换 |

早期 CONSENSUS「不做仿真/持久化」等表述已过时，以插件 README 与源码为准。
