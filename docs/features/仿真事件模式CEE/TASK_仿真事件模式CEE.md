# TASK — 仿真事件模式（薄 CEE）

| ID | 任务 | 依赖 |
|----|------|------|
| T1 | 文档 + Bug #68 | — |
| T2 | RobotScene Cee* + Kernel 挂钩 + 自测 | T1 |
| T3 | devicePrograms 工程 IO | T1 |
| T4 | Controller EventBased Run + Handler + Driver 开关 | T2 |
| T5 | UI 模式 / 规则表 / bindings 同步 | T4 |
| T6 | simulationRunMode / ceeRules 工程 IO | T5 |
| T7 | ACCEPTANCE + 双配置编译 | T2–T6 |
