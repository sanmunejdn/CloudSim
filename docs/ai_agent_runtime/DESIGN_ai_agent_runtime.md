# DESIGN — Full AI Agent Runtime

```mermaid
flowchart TB
  User[用户消息] --> Coord[AiAssistantCoordinator]
  Coord -->|catalog 域| Agent[AiAgentRuntime]
  Agent --> Snap[AiSceneSnapshotBuilder]
  Agent --> Mem[AiAgentMemory]
  Agent --> Think[rules 或 chatWithTools]
  Think --> Panel[AiConfirmPanel]
  Panel -->|确认| Exec[AiHostButtonApiDispatch allowModal=false]
  Exec --> Obs[观测写入 memory]
  Obs --> Agent
  Agent --> Reply[对话回复]
  Coord -->|recognize/trajectory| Panel
```

## 分层

| 层 | 组件 |
|----|------|
| SDK | `AiAgentTypes`、`IAiAssistantHost` Agent API |
| Host | Runtime、Memory、Snapshot、LlmClient tools、Dispatch |
| Widget | ConfirmPanel、Dock、Coordinator pending |

## 事件

`NeedConfirm` | `StepDone` | `Finished` | `Error` — Host `invokeOnUiThread` 投递

## 闸门

- `args_schema` 非空 → 必须面板
- `risk=low` 且空 schema → 可直跑
- `medium`/`high` → 面板 + 风险文案；Agent 路径禁止模态补参
