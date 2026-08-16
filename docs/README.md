# CloudSim 文档索引

**日常开发以本节「常读」与各模块 `DEVELOPER_GUIDE.md` 为准。** 已完成的 6A 专题在 [`_archive/`](_archive/)；冲突时以源码与模块指南为准，不必回溯改写全部归档文。

## 常读（与当前代码一致）

| 文档 | 说明 |
|------|------|
| [`DIRECTORY_LAYOUT.md`](DIRECTORY_LAYOUT.md) | `src/` 域划分、工程对照、构建输出 |
| [`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md) | 各模块 `DEVELOPER_GUIDE.md` 索引 |
| [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md) | 编码、头卫、文件头、clang-format、筛选器 |
| [`spatial_contract_world_pose.md`](spatial_contract_world_pose.md) | 世界坐标 / `worldMatrix` 契约 |
| [`后端对象与软件模式/README.md`](后端对象与软件模式/README.md) | 后端类型三键、侧车键、工作区模式 vs Data |
| [`几何建模/`](几何建模/) | 几何建模功能清单、架构、路线图 |
| [`HostOptimization/`](HostOptimization/) | 接口目录、backend 调用清单、Headless 运维（6A 过程稿已归档） |
| [`web/cloudsim-web-ui/DEVELOPER_GUIDE.md`](../web/cloudsim-web-ui/DEVELOPER_GUIDE.md) | 网页正式壳：构建、场景、轨迹、IO 网络与自定义设备 |
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

## 历史归档

索引见 [`_archive/INDEX.md`](_archive/INDEX.md)。近期迁入示例：自定义设备系列、IO 信号与流程、网页端 IO/坐标系/React 轨迹、点云网页同步、轨迹离散 AI 确认、HostOptimization 6A、AI 助手计划校验单轨。

> 原根目录 `ARCHITECTURE_SUMMARY.md` 已删除；架构入口为本索引 + `MODULE_DEVELOPER_GUIDES.md` + 各 `DEVELOPER_GUIDE.md`。

## 源码格式维护命令（摘要）

在 `CloudSim/` 根目录：

```bash
python scripts/run_clang_format.py
python scripts/normalize_source_encoding.py
python scripts/generate_vcxproj_filters.py --sync
```

完整约定见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。
