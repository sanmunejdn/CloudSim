# OPTIONAL_EVAL — Web H2 / AiHost

## Web H2：链接期去 OSG

| 项 | 说明 |
|----|------|
| 诉求 | `CloudSimWeb` 进程不链 `OsgWidgetCore` / BackendVisual |
| 现状 | 运行期已 `createHeadlessDocumentHost(..., false)`；链接期仍可能带渲染栈 |
| 判定 | Phase1 **已落地**（`CloudSimHostHeadless`） |
| 入口 | `ALIGNMENT_WebH2_链接期去OSG.md` / `ACCEPTANCE_WebH2_链接期去OSG.md` |
| 状态 | Phase1 完成（2026-08-14 复验 T4 仍过）；Phase2（RobotUrdf/BackendVisual 去 osg 传递）仍延期 |

## AiHost 物理拆 DLL

| 项 | 说明 |
|----|------|
| 诉求 | AI HTTPS/LLM 与场景 DLL 隔离、可剥离安装 |
| 现状 | `CloudSimPluginHost/Ai/*` 编入 `CloudSimHost.dll` |
| 判定 | **仅有独立发布/依赖隔离诉求再拆** |
| 状态 | **延期**（路径 B：逻辑边界已在 INTERFACE_CATALOG §H.2） |

## 明确不做

- 拆 `CloudSimPluginHost.dll` / Osg 壳独立 DLL
