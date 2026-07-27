# ALIGNMENT — 工艺流程 AI 助手

## 原始需求

开放 AI 助手页面；用户输入场景描述，助手自动生成工艺流程并仿真。

## 项目理解

- AI Agent 仅经 Host Catalog/`AiHostButtonApiDispatch` 调 Host 接口
- 工艺流程画布/DES 在 `ProcessFlowPlugin` 私有域
- 进入工艺流程时原 `m_unitDock`（含 AI）被整栏隐藏

## 边界

- 做：流程模式露出 AI；`IProcessFlowAiBridge`；域 `process.flow`；整图 JSON 写入 + 同步 DES
- 不做：逐节点拖拽式 AI 编辑、CP-SAT/RL、强制开启 remote_llm、机器人 binding

## 疑问（已决策）

| 项 | 决策 |
|----|------|
| 开放方式 | 与仿真右侧栏 tabify，仅保留 AI 页签 |
| 生成方式 | Agent 工具：applyGraph → runSim；规则模板兜底 |
| Host 接入 | 插件注册 `IProcessFlowAiBridge`，ABI 1.20.0 |
