# CloudSim 文档索引

> **全仓库统一入口**（手册 + 全量目录）：[`../../docs/README.md`](../../docs/README.md) · [`../../docs/全量目录.md`](../../docs/全量目录.md)

**日常开发以本节与各模块 `DEVELOPER_GUIDE.md` 为准。** 已完成的 6A 专题在 [`_archive/`](_archive/)；冲突时以源码与模块指南为准。

## 目录分层（整理后）

| 层 | 内容 |
|----|------|
| 常读 | 本 README、布局/约定/契约、模式入口、Host 优化、架构图、后端-OSG、路径规划 PLANNERS、指令 IK DEVELOPER |
| 领域资产 | [`设计计算/`](设计计算/)、[`标准件/`](标准件/)、[`运动副/`](运动副/) |
| 进行中 | 拓扑命名、自定义设备旋转中心/挂载、网页全量对等 |
| 工具 | [`文档工具/`](文档工具/)（断链扫描等） |
| 历史 | [`_archive/`](_archive/) |

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

完整表见 [`插件/`](插件/)；源码侧索引见 [`../src/Plugins/README.md`](../src/Plugins/README.md)。

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
| [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md) | 编码、头卫、clang-format、筛选器 |
| [`spatial_contract_world_pose.md`](spatial_contract_world_pose.md) | 世界坐标 / `worldMatrix` 契约 |
| [`后端对象与软件模式/README.md`](后端对象与软件模式/README.md) | 后端类型三键、侧车键、工作区模式 vs Data |
| [`Host优化/`](Host优化/) | 接口目录、backend 调用清单、Headless 运维 |
| [`架构/`](架构/) | 桌面 / 网页架构 HTML |
| [`后端OSG同步/`](后端OSG同步/) | 后端 ↔ OSG |
| [`后端属性Binding/`](后端属性Binding/) | Data 面板/schema Binding（typed API 真源） |
| [`指令与插件属性Binding/`](指令与插件属性Binding/) | RobotInstruction Binding + 插件 Path B 注册表 |
| [`机器人路径规划/`](机器人路径规划/) | 规划算法原理（常读）；6A 过程稿在 [`_archive/机器人路径规划/`](_archive/机器人路径规划/) |
| [`指令IK轨迹重构/`](指令IK轨迹重构/) | `DEVELOPER_指令IK轨迹.md`（常读）；6A 在 [`_archive/指令IK轨迹重构/`](_archive/指令IK轨迹重构/) |
| [`设计计算/`](设计计算/)、[`标准件/`](标准件/) | 设计计算 / 标准件领域资产 |
| [`web/cloudsim-web-ui/DEVELOPER_GUIDE.md`](../web/cloudsim-web-ui/DEVELOPER_GUIDE.md) | 网页正式壳 |
| [`src/Web/CloudSimWebGateway/DEVELOPER_GUIDE.md`](../src/Web/CloudSimWebGateway/DEVELOPER_GUIDE.md) | 网页 Gateway |

Cursor 规则：`.cursor/rules/cloudsim-cpp-conventions.mdc`、`cloudsim-architecture.mdc`、`cloudsim-vcxproj-filters.mdc`、`vs-build-configurations.mdc`。

## 近期热点（改相关代码前先看）

| 主题 | 入口 |
|------|------|
| Units 多文档树 / 展开三角 / `BackendUnitsTreeBinder` | [`Widget/DEVELOPER_GUIDE.md`](../src/UI/Widget/DEVELOPER_GUIDE.md) §5.3、§4.6 |
| 多文档 Tab 开工程 / IO 网络缓存 | [`Widget/DEVELOPER_GUIDE.md`](../src/UI/Widget/DEVELOPER_GUIDE.md) §6、§5.3 |
| URDF 空壳根保存再开 | [`CloudSimHost/DEVELOPER_GUIDE.md`](../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) §4.4.4 / §4.2c |
| TCP 拖动示教与罗盘 | [`RobotWidget/DEVELOPER_GUIDE.md`](../src/UI/RobotWidget/DEVELOPER_GUIDE.md)、[`OsgWidgetCore`](../src/UI/OsgWidgetCore/DEVELOPER_GUIDE.md) |
| IO 信号网络 / 连接站 / 自定义设备 | 桌面：[`RobotWidget`](../src/UI/RobotWidget/DEVELOPER_GUIDE.md)；网页指南见 web-ui；过程稿 [`_archive/IO信号与流程/`](_archive/IO信号与流程/)、[`_archive/网页端信号网络与自定义设备/`](_archive/网页端信号网络与自定义设备/) |
| Robot 运动学 Workspace | [`RobotUrdf`](../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md)；图与 6A：[`_archive/robot-kinematics-workspace/`](_archive/robot-kinematics-workspace/) |
| **运动副（1-DOF FK）** | [`运动副/`](运动副/) · [架构图](运动副/运动副架构图.html) |
| **自定义设备旋转中心 / 机器人挂载** | [`自定义设备旋转中心Frame/`](自定义设备旋转中心Frame/) · [`自定义设备机器人挂载/`](自定义设备机器人挂载/) |
| Follow / Compound | [`_archive/Follow与Compound分流/`](_archive/Follow与Compound分流/) |
| 视口拾取 | [`_archive/视口拾取重构/`](_archive/视口拾取重构/) |
| **属性 Binding（Data / 指令 / 插件）** | [`后端属性Binding/`](后端属性Binding/) · [`指令与插件属性Binding/`](指令与插件属性Binding/) · [`Data` §5](../src/Data/Data/DEVELOPER_GUIDE.md) · [`RobotScene` §10](../src/Robot/RobotScene/DEVELOPER_GUIDE.md) · [`CloudSimPluginSDK`](../src/Plugins/CloudSimPluginSDK/DEVELOPER_GUIDE.md) |

## 进行中专题

| 文档 | 说明 |
|------|------|
| [`拓扑命名/`](拓扑命名/) | TopoNaming 对齐（本里程碑未实现） |
| [`自定义设备旋转中心Frame/`](自定义设备旋转中心Frame/) | 旋转中心 Frame 绑定与 applyQ 视觉回写 |
| [`自定义设备机器人挂载/`](自定义设备机器人挂载/) | 法兰挂载、TCP 跟踪与 FK 后刷新 |
| [`运动副/`](运动副/) | 1-DOF 运动副设计、FK 组合与架构图 |
| [`网页端全量对等/`](网页端全量对等/) | API_CONTRACT、AGENT_BRANCHES |

## 本轮已迁入归档（活跃区不再保留）

| 原活跃目录 | 现路径 |
|------------|--------|
| Follow与Compound分流 | [`_archive/Follow与Compound分流/`](_archive/Follow与Compound分流/) |
| UI思想落地 | [`_archive/UI思想落地/`](_archive/UI思想落地/) |
| 视口拾取重构 | [`_archive/视口拾取重构/`](_archive/视口拾取重构/) |
| 机器人挂载跟随修复 | [`_archive/机器人挂载跟随修复/`](_archive/机器人挂载跟随修复/) |
| 网页端机器人页完整同步 | [`_archive/网页端机器人页完整同步/`](_archive/网页端机器人页完整同步/) |
| 网页端主程序壳对等 | [`_archive/网页端主程序壳对等/`](_archive/网页端主程序壳对等/) |
| 网页端指令IK播放对等 | [`_archive/网页端指令IK播放对等/`](_archive/网页端指令IK播放对等/) |
| HostFiltersAndApiDocs | [`_archive/HostFiltersAndApiDocs/`](_archive/HostFiltersAndApiDocs/) |
| 开发文档整理（6A） | [`_archive/开发文档整理/`](_archive/开发文档整理/)；脚本见 [`文档工具/`](文档工具/) |
| 机器人路径规划 6A | [`_archive/机器人路径规划/`](_archive/机器人路径规划/)（活跃区仅留 PLANNERS） |
| 指令IK轨迹重构 6A | [`_archive/指令IK轨迹重构/`](_archive/指令IK轨迹重构/)（活跃区仅留 DEVELOPER） |
| `large_nonrigid_registration_acceleration_framework.html` | [`_archive/`](_archive/large_nonrigid_registration_acceleration_framework.html) |

## 历史归档

完整表见 [`_archive/INDEX.md`](_archive/INDEX.md)。活跃区勿再链到已迁入 `_archive` 的同名目录。

## 源码格式维护命令（摘要）

在 `CloudSim/` 根目录：

```bash
python scripts/run_clang_format.py
python scripts/normalize_source_encoding.py
python scripts/generate_vcxproj_filters.py --sync
```

断链抽查（排除 third_party / `_archive`）：

```bash
python docs/文档工具/_scan_links_active.py
```

完整约定见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。
