# ACCEPTANCE — AI 助手计划校验 / 观测 / 确认单轨

| 项 | 状态 | 备注 |
|----|------|------|
| args_schema 确认后校验 | 通过（编译） | `AiArgsSchema::missingRequiredArgs` → Runtime 执行前 |
| mesh.compose 执行前校验 | 通过（编译） | 与 feature.compose 对称 |
| 快照差分 newBackendIds | 通过（编译） | 观测串可含 `new=` |
| 轨迹确认经 Runtime | 通过（编译） | NeedConfirm → 模态对话框；Accept→submit；返回重选→Secondary 保会话 |
| CloudSimHost Debug\|x64 | 通过 | |
| CloudSimHost Release\|x64 | 通过 | |
| AiWidget Debug\|x64 | 通过 | |
| AiWidget Release\|x64 | 通过 | |

## 建议手测

1. Catalog：「体素下采样」→ 面板补参 → 确认 → 观测含对象变化/`new=`（若新建对象）。
2. 故意清空必填参确认 → 提示「缺少必填参数：…」。
3. 轨迹：识别 → 选 N →「确认」→ 离散对话框 → 确认离散成功；「返回重选」仍可「选 N」；取消后选择仍在。
