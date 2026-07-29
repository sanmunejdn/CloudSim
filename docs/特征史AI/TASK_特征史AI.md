# TASK — feature.compose

| ID | 任务 | 验收 |
|----|------|------|
| T1 | AiDomainIds + FeatureComposeDomainHandler + 注册 | domain 可 resolve/execute |
| T2 | Router + LLM prompt | CAD 词 → feature.compose |
| T3 | Executor 三 API + waitGeom | Pad 写入 history |
| T4 | Host 回调 + Plugin sync | 树刷新 |
| T5 | 规则解析简单板件 | 无 LLM 也可 Pad |
| T6 | 文档 + 编译 | ACCEPTANCE |

依赖：T1→T2/T3；T3→T4；并行 T5。
