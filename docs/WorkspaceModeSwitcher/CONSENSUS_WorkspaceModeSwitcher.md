# CONSENSUS — WorkspaceModeSwitcher

## 需求描述

在菜单栏下提供宿主级分段控件，一键切换：主程序 / 几何建模 / 工艺流程 / 工程图。选中态与 `currentWorkspaceMode()` 一致。

## 验收标准

1. 顶栏 `WorkspaceModeBar` 始终可见；选中项与当前 mode 同步。
2. 四向切换（插件已加载时）无卡死、无双 Ribbon、无侧栏残留。
3. Light/Dark 可读；中英文随语言切换；`Ctrl+1..4` 快捷键。
4. 插件菜单与分段走同一 enter/exit / `returnToMainWorkspace`。
5. Debug|x64 与 Release|x64 编译通过（SDK、Host、Widget、三插件）。

## 技术方案

- SDK ABI bump → `0x00012300`（1.35.0）：`registerWorkspaceMode`、`returnToMainWorkspace`、`workspaceModeEntries`（或等价查询）。
- `PluginHostContext` 存模式表；内建主程序；修 claim 清 Ribbon。
- `WorkspaceModeSwitcher` + `MainWindow` 接线；`ApplicationStyle` 青绿选中。
- 三插件 `initialize` 注册；drawing softExit 对齐「只清本地」。

## 技术约束

- vtable 仅末尾追加；与现有 `claimWorkspaceMode` / softExit 协作。
- 强调色 `#0f766e`（对齐几何/工程图 Ribbon），非 AI 紫。

## 任务边界

不做 Rail/Dock；不改特征树/流程节点/出图算法；不删菜单。
