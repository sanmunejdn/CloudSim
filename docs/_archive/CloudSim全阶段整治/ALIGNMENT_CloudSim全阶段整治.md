# ALIGNMENT — CloudSim 全阶段整治

## 原始需求

按「清理 → 静态排 bug → Top3 定点修 → Debug|x64 + Release|x64 双编」整治 `CloudSim/`：删除安全垃圾、归档历史 docs、多 agent 静态审计、证据驱动修复 Top3 缺陷。

## 项目上下文

- 架构权威：[文档索引](../../README.md)
- 目录权威：[`DIRECTORY_LAYOUT.md`](../../DIRECTORY_LAYOUT.md)
- 常读索引：[docs/README.md](../../README.md)
- 构建：`CloudSim.sln` + `Directory.Build.props` → 仓库根 `bin\x64(d)/`

## 任务边界

### 范围内

1. 安全垃圾清理（日志、错误位置 `bin`/`x64`、`.venv`、`__pycache__`、第三方 logs）
2. 历史专题 docs 整目录迁入 `docs/_archive/`
3. 四路静态审计 → `BUG_AUDIT.md`
4. Top3 定点修（见 CONSENSUS）
5. 触及工程 Debug + Release 双编

### 范围外

- 架构债重构（`backend()` 穿透、OSG 全面解耦、`RobotSimulationController` 迁 Host）
- 删除 `third_party/`
- 改 `OutDir` / 产物路径
- Top3 以外的功能扩展（转台外轴、blend、品牌 ARC、实现 RemeshSoup 等）

## Top3（证据驱动）

| # | 主题 | 锚点 |
|---|------|------|
| T1 | 轨迹编辑拖放 / `m_ops` 同步 | `TrajectoryEditPageWidget.cpp` |
| T2 | Run / 外轴 / 插帧路径确认缺陷 | `RobotSimulationController` / `RobotInstructionController` |
| T3 | Mesh 未实现模式防护 + 关键 I/O 空 catch 可观测 | `MeshDiscretize` + 工程打开/运动学 restore |

## 删除白名单

| 类别 | 允许 |
|------|------|
| 构建/调试日志 | 是 |
| `src/**/bin`、`src/**/x64`、工具 `obj` | 是 |
| `.venv` / `__pycache__` / vendor logs | 是 |
| 历史 docs → `_archive/` | 是（搬迁非散删） |
| HelloAiPlugin / Sketch*Verify / PluginHost.vcxproj | **否**（保留） |
| 无文档引用的一次性 `tools/gen_*.py` | 盘点后移 `_archive` 或删 |

## 常读保留（不归档）

`DIRECTORY_LAYOUT.md`、`MODULE_DEVELOPER_GUIDES.md`、`SOURCE_CONVENTIONS.md`、`spatial_contract_world_pose.md`、`README.md`、`后端对象与软件模式/`、本目录 `CloudSim全阶段整治/`。

## 疑问澄清

无未决决策；按计划执行。
