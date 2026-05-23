# 后端持久化文档索引

本目录记录 `project.json` **v4** 及 `BackendDataBase` 多态序列化相关的设计、任务与回归说明。

| 文档 | 用途 |
|------|------|
| [DESIGN_backend_persistence.md](DESIGN_backend_persistence.md) | 架构：基类模板方法、注册表、数据契约 |
| [PROPERTY_PERSISTENCE_OPTIONS.md](PROPERTY_PERSISTENCE_OPTIONS.md) | 属性（PropertyBag）保存方案对比 |
| [TASK_backend_persistence.md](TASK_backend_persistence.md) | 原子任务与依赖 |
| [ACCEPTANCE_backend_persistence.md](ACCEPTANCE_backend_persistence.md) | 落地进度与 DoD |
| [REGRESSION_CHECKLIST_backend_persistence.md](REGRESSION_CHECKLIST_backend_persistence.md) | 构建与保存/恢复手工回归 |

**代码入口**：

- `src/Data/Data` — `BackendDataBase::saveToJson` / `loadFromJson`，`BackendComponentCodecBuiltins.h`
- `src/UI/Widget/source/MainWindowProjectIo.cpp` — 文档保存/打开编排

**开发文档**：

- [Data DEVELOPER_GUIDE.md](../../src/Data/Data/DEVELOPER_GUIDE.md) §3.8、§6.1、§9
- [Widget DEVELOPER_GUIDE.md](../../src/UI/Widget/DEVELOPER_GUIDE.md) §11
- [ARCHITECTURE_SUMMARY.md](../../ARCHITECTURE_SUMMARY.md) §6.5
