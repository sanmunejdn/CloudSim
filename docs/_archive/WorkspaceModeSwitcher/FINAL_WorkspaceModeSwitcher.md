# FINAL — WorkspaceModeSwitcher

## 交付摘要

宿主顶栏分段控件统一切换主程序 / 几何建模 / 工艺流程 / 工程图，替代分散菜单作为主交互。

## 关键变更

| 区域 | 内容 |
|------|------|
| SDK | ABI `0x00012300`（1.35.0）：`registerWorkspaceMode` / `returnToMainWorkspace` / `enterWorkspaceMode` |
| Host | 模式注册表；修 claim 清 Ribbon；`notifyWorkspaceModesChanged` |
| Widget | `WorkspaceModeSwitcher` + `WorkspaceModeBar`；`setModeToolBar` 互斥显隐；ApplicationStyle |
| 插件 | 三插件注册模式；exit/菜单走 `returnToMainWorkspace`；drawing softExit 对齐 |

## 编译

Debug\|x64 与 Release\|x64 均已通过：CloudSimPluginSDK → CloudSimHost → Widget → GeometricModeling / ProcessFlow / EngineeringDrawing。
