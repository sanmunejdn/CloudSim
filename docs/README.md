# CloudSim 文档索引

总架构见仓库根 [`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)。本目录按主题存放设计/任务/验收文档；**日常开发约定以本节「常读」为准**。

## 常读（与当前代码一致）

| 文档 | 说明 |
|------|------|
| [`DIRECTORY_LAYOUT.md`](DIRECTORY_LAYOUT.md) | `src/` 域划分、工程对照、构建输出 |
| [`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md) | 各模块 `DEVELOPER_GUIDE.md` 索引 + 导出/筛选器/注释入口 |
| [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md) | **源码格式权威约定**：编码、头卫、文件头、clang-format、筛选器脚本 |
| [`spatial_contract_world_pose.md`](spatial_contract_world_pose.md) | 世界坐标 / `worldMatrix` 契约（改空间相关代码前必读） |
| [`后端对象与软件模式/`](后端对象与软件模式/) | 后端类型三键、侧车键、工作区模式 vs Data（P0 契约） |
| [`CloudSim全阶段整治/`](CloudSim全阶段整治/) | 目录清理、静态审计、Top3 定点修（已验收，见 FINAL） |

Cursor 规则镜像：`.cursor/rules/cloudsim-cpp-conventions.mdc`、`cloudsim-architecture.mdc`、`cloudsim-vcxproj-filters.mdc`。

## 历史归档

专题任务文档已迁入 [`_archive/`](_archive/)，索引见 [`_archive/INDEX.md`](_archive/INDEX.md)。

历史任务文档中的接口描述若与现网代码冲突，**以 `src/**/DEVELOPER_GUIDE.md` 与常读文档为准**，不必回溯改写全部归档文。

## 源码格式维护命令（摘要）

在 `CloudSim/` 根目录：

```bash
python scripts/run_clang_format.py
python scripts/normalize_source_encoding.py
python scripts/generate_vcxproj_filters.py --sync
```

完整约定与脚本列表见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。
筛选器规则（Cursor）：`.cursor/rules/cloudsim-vcxproj-filters.mdc`。