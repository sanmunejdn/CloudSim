# CONSENSUS — 开发文档整理

## 已确认决策

| 项 | 选择 |
|----|------|
| 整理方式 | **A**：不搬 `_archive`；索引 + 短入口页 |
| 范围 | **A**：`CloudSim/docs` + `CloudSim/README.md`；根 `docs/` 仅加指向 |
| 工程图/工艺流程 | **A**：各一页活跃 README，不补 FEATURES 套件 |
| 落地优先级 | P0 断链+模式/插件表 → P1 两模式 hub → P2 根 docs + 清理过期引用 |

## 验收标准

1. `几何建模`、`ProcessFlowPlugin` 等已知断链指向 `_archive/...` 或正确活跃路径
2. `CloudSim/docs/README.md` 含「按软件模式」「按插件类型」两段表
3. 存在 `工程图/README.md`、`工艺流程/README.md`（及便于模式表落地的 `主程序/README.md`）
4. 仓库根 `docs/README.md` 说明真源在 `CloudSim/docs`
5. 过期「进行中」/把已归档当活跃的交叉引用已清理
