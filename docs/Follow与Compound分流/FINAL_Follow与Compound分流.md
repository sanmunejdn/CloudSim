# FINAL — Follow 与 Compound 分流

## 交付摘要

按「全面分流」落地：Follow 仅跨部件；同部件靠 Data 父子 + compound 刚体 Δ。

## 主要代码

| 模块 | 变更 |
|------|------|
| Data | `BackendCompoundPropagate` |
| RobotScene | `applyToSink` 复用 compound |
| Host | 切断 attach/edges 自动 Follow；`stripHierarchyDriven`；Follow→compound→二轮 Follow→挂载 refresh |
| Widget | 工程加载 legacy Follow 回调 no-op |

## 文档

- `docs/Follow与Compound分流/CONSENSUS_*.md` / `ACCEPTANCE_*.md`
- Host / Data / RobotScene / RobotWidget DEVELOPER_GUIDE
- `docs/自定义设备机器人挂载/DESIGN_*.md`

## 编译

Debug\|x64 与 Release\|x64：Data、RobotScene、CloudSimHost、Headless、Widget 已通过。
