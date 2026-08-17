# CONSENSUS — 开发文档全面整理（第二轮）

## 决策

| 项 | 选择 |
|----|------|
| 过时「删除」 | **已完成 6A 专题迁入 `_archive`**（不物理销毁历史稿）；仅删除临时扫描产物与明确无用 stub |
| `src` 文档 | 保留各模块 `DEVELOPER_GUIDE.md` / 插件 `README.md`；统一相对路径；补缺省短 README |
| `_archive` | 保留；索引更新；活跃区不再挂已验收专题 |
| third_party / node_modules | **不整理** |

## 活跃区保留

- 模式/插件导航 hub、横切常读、`几何建模/` 现状文档
- `HostOptimization/` 运维清单、`TopoNaming/`、`自定义设备旋转中心Frame/`
- `design.calc/`、`design.parts/`（领域资产，非已归档过程稿）

## 迁入 `_archive`

- `robot-kinematics-workspace/`（已有 FINAL）
- `网页端信号网络与自定义设备/`（ACCEPTANCE 全勾）
- `网页端设备页桌面同步/`（ACCEPTANCE 通过）

## 验收

1. 一等公民 md（docs 活跃 + src 指南，排除 third_party）断链清零或仅剩「工具路径可选缺失」
2. 总索引与模块索引路径正确
3. `src` 内跨模块 / 指向 docs 的链接使用正确 `../` 深度与 `_archive` 前缀
