# CloudSim 文档索引

**日常开发以本节与各模块 `DEVELOPER_GUIDE.md` 为准。** 已完成的 6A 专题在 [`_archive/`](_archive/)；冲突时以源码与模块指南为准，不必回溯改写全部归档文。

按**软件模式**或**插件类型**查找时，优先用下面两段表；横切约定仍见「常读」。

## 按软件模式

顶栏工作区：主程序（内建）→ 几何建模 → 工艺流程 → 工程图。

| 模式 | modeId | 活跃入口 | 代码 / 指南 |
|------|--------|----------|-------------|
| 主程序 | `""`（空） | [`主程序/`](主程序/) | Widget / Host / RobotWidget / Data |
| 几何建模 | `com.cloudsim.geomodeling` | [`几何建模/`](几何建模/) | [GeometricModelingPlugin](../src/Plugins/GeometricModelingPlugin/README.md) |
| 工艺流程 | `com.cloudsim.processflow` | [`工艺流程/`](工艺流程/) | [ProcessFlowPlugin](../src/Plugins/ProcessFlowPlugin/README.md) |
| 工程图 | `com.cloudsim.drawing` | [`工程图/`](工程图/) | [EngineeringDrawingPlugin](../src/Plugins/EngineeringDrawingPlugin/README.md) |

模式切换过程稿：[`_archive/WorkspaceModeSwitcher/`](_archive/WorkspaceModeSwitcher/)。契约：[`后端对象与软件模式/`](后端对象与软件模式/)。

## 按插件类型

完整表见 [`插件/`](插件/)。摘要：

| 类型 | 成员 |
|------|------|
| 工作区模式 | GeometricModeling / ProcessFlow / EngineeringDrawing |
| 侧栏工具 | Geometry / PointCloud / PlcComm / IndustrialCamera / Labeling / PointNet / HelloAi |
| AI | CloudSimAiSDK、AiWidget、PointNet、HelloAi |
| 轨迹 / 通讯 SDK | CloudSimMeshTrajectorySDK、RobotCommSDK |
| 契约 SDK | CloudSimPluginSDK |

## 常读（与当前代码一致）

| 文档 | 说明 |
|------|------|
| [`DIRECTORY_LAYOUT.md`](DIRECTORY_LAYOUT.md) | `src/` 域划分、工程对照、构建输出 |
| [`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md) | 各模块 `DEVELOPER_GUIDE.md` 索引 |
| [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md) | 编码、头卫、文件头、clang-format、筛选器；注释见 MODULE 指南 |
| [`spatial_contract_world_pose.md`](spatial_contract_world_pose.md) | 世界坐标 / `worldMatrix` 契约 |
| [`后端对象与软件模式/README.md`](后端对象与软件模式/README.md) | 后端类型三键、侧车键、工作区模式 vs Data |
| [`HostOptimization/`](HostOptimization/) | 接口目录、backend 调用清单、Headless 运维（6A 过程稿已归档） |
| [`web/cloudsim-web-ui/DEVELOPER_GUIDE.md`](../web/cloudsim-web-ui/DEVELOPER_GUIDE.md) | 网页正式壳：构建、场景、轨迹、IO 网络与自定义设备；WebGL dispose |
| [`src/Web/CloudSimWebGateway/DEVELOPER_GUIDE.md`](../src/Web/CloudSimWebGateway/DEVELOPER_GUIDE.md) | 网页 Gateway：REST/SSE、事件队列已知风险 |
| [`网页端信号网络与自定义设备/`](网页端信号网络与自定义设备/) | 网页多 Owner 连接站 + 自定义设备 API/运行面 |
| [`网页端设备页桌面同步/`](网页端设备页桌面同步/) | 右栏「设备」模式切换 + 设备指令页对齐桌面 |

Cursor 规则：`.cursor/rules/cloudsim-cpp-conventions.mdc`、`cloudsim-architecture.mdc`、`cloudsim-vcxproj-filters.mdc`、`vs-build-configurations.mdc`。

## 近期热点（改相关代码前先看）

| 主题 | 入口 |
|------|------|
| Units 多文档树 / 展开三角 / `BackendUnitsTreeBinder` | [`Widget/DEVELOPER_GUIDE.md`](../src/UI/Widget/DEVELOPER_GUIDE.md) §5.3、§4.6 |
| 多文档 Tab 开工程 / IO 网络缓存 | [`Widget/DEVELOPER_GUIDE.md`](../src/UI/Widget/DEVELOPER_GUIDE.md) §6、§5.3 |
| URDF 空壳根保存再开 | [`CloudSimHost/DEVELOPER_GUIDE.md`](../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) §4.4.4 / §4.2c |
| TCP 拖动示教与罗盘 | [`RobotWidget/DEVELOPER_GUIDE.md`](../src/UI/RobotWidget/DEVELOPER_GUIDE.md)、[`OsgWidgetCore`](../src/UI/OsgWidgetCore/DEVELOPER_GUIDE.md) |
| IO 信号网络 / 连接站 / 自定义设备绑定 | 桌面：[`RobotWidget/DEVELOPER_GUIDE.md`](../src/UI/RobotWidget/DEVELOPER_GUIDE.md)；网页：[`网页端信号网络与自定义设备/`](网页端信号网络与自定义设备/)；过程稿 [`_archive/IO信号与流程/`](_archive/IO信号与流程/) |

## 进行中专题

| 文档 | 说明 |
|------|------|
| [`TopoNaming/`](TopoNaming/) | TopoNaming 对齐（本里程碑未实现） |
| [`自定义设备旋转中心Frame/`](自定义设备旋转中心Frame/) | 自定义设备旋转中心 Frame（ALIGNMENT） |
| [`开发文档整理/`](开发文档整理/) | 文档按模式/插件导航整理（本轮） |

## 历史归档

索引见 [`_archive/INDEX.md`](_archive/INDEX.md)。几何建模相关专题（草图硬化、SW 差距、硬化基准面、特征史 AI、WorkspaceModeSwitcher 等）均在 `_archive/`，勿再链到已删除的顶层同名目录。

> 原根目录 `ARCHITECTURE_SUMMARY.md` 已删除；架构入口为本索引 + `MODULE_DEVELOPER_GUIDES.md` + 各 `DEVELOPER_GUIDE.md`。

## 源码格式维护命令（摘要）

在 `CloudSim/` 根目录：

```bash
python scripts/run_clang_format.py
python scripts/normalize_source_encoding.py
python scripts/generate_vcxproj_filters.py --sync
```

完整约定见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。
