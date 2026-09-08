# ACCEPTANCE：指令与插件属性 Binding

## 实现对照

| 项 | 状态 |
|----|------|
| Instruction Binding；删 Attribute/`m_attributes`；schema 由 Binding 生成 | 通过 |
| 未知 key 失败；extension 不进 snapshot/apply | 通过 |
| 面板按 Type 查 descriptor；SelfTest | 通过 |
| SDK `PluginPropertyBindingEntry`；版本 bump `0x00013600`（1.54.0） | 通过 |
| Data `BackendExternalPropertySchemaRegistry` + schema cache invalidate | 通过 |
| PluginDelegatedBackend：有 Binding 走基类+插件行；无 Binding 回退 JSON | 通过 |
| Host mock SelfTest | 通过 |
| Debug\|x64 + Release\|x64 构建链 | 通过（2026-09-07） |

## 明确未做

- sidecar 插件改 Path B
- Instruction 并入 IDataService
- 插件直接链 Data Binding 头
