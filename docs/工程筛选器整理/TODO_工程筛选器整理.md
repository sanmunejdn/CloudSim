# TODO — 工程筛选器整理

## 待你本地确认

1. **用 VS 打开 `CloudSim.sln`**，抽查大工程筛选器树是否符合预期（Builtins / Widget / GeometryAlgorithm / Host）
2. 若某文件归类不合意：改 `tools/RegenerateProjectFilters.ps1` 中对应工程的匹配规则，再重跑脚本（勿手改大量 filters，易被覆盖）

## 可选后续（未做）

| 项 | 说明 |
|----|------|
| 物理目录对齐 | 当前仅改 Solution Explorer 视图；若希望磁盘也按功能分子目录，需另开任务（会动 include 路径） |
| Widget 拾取 cpp | 部分 Pick/Osg 实现编在 Host 的 External 引用里，不在 Widget.vcxproj；若要 Widget 本地也能看到，需改 vcxproj 归属 |
| SLN 外工程 | `HelloAiPlugin`、`CloudSimPluginHost` 未纳入；需要时可对脚本加路径或单独跑 |
| GUID 稳定性 | 每次全量重跑会为**新**筛选器名生成新 GUID；已有同名 Filter 会尽量保留旧 GUID |

## 已处理（本轮补充）

- `CloudSimPluginHost`：`inc\Ai\{Agent,Llm,Domains,Catalog,Rules,Core,Global}` + `PluginHost` / `Document` / `Interfaces`
- `HelloAiPlugin`：`inc\Plugin` / `src\Plugin`
- 脚本默认 `ExtraProjects` 已包含上述两者；全量重跑也会处理

## 配置 / 支持

- 无额外环境配置
- 脚本依赖 PowerShell 5.1+ / XML 解析，无第三方模块
