# ACCEPTANCE — WorkspaceModeSwitcher

| # | 验收项 | 结果 |
|---|--------|------|
| 1 | 顶栏 `WorkspaceModeBar` 分段始终可见 | 通过（代码+编译） |
| 2 | 选中与 `currentWorkspaceMode()` 同步（主=`""`） | 通过（claim 回调 → setCurrentMode） |
| 3 | 插件 register 后分段补齐；点击走 `enterWorkspaceMode` | 通过 |
| 4 | `returnToMainWorkspace` 清 Ribbon/3D/侧栏 | 通过 |
| 5 | softExit 不抢中央 3D；Ribbon 由 softExit/return 清理 | 通过 |
| 6 | claim 不再硬清非几何 Ribbon | 通过 |
| 7 | Light/Dark QSS `#WorkspaceModeBar` 青绿选中 | 通过 |
| 8 | Debug\|x64 + Release\|x64：SDK/Host/Widget/三插件 | 通过 |

手动建议：启动后点四分段互切，确认无双 Ribbon、无侧栏残留。
