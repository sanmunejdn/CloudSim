# ALIGNMENT — WorkspaceModeSwitcher

## 原始需求

设计新的交互方式，在几何建模、工艺流程、工程图与主程序之间切换。选定方案 2：菜单栏下顶栏分段控件。

## 项目上下文

- CloudSim：Qt 桌面工业机器人仿真；UI 编排在 `Widget.dll`，插件经 `IPluginHostContext` 互斥工作区。
- 现状：各插件菜单「进入 / 返回三维场景」+ `claimWorkspaceMode(QString)`；无全局切换器。
- 已有 `ModeToolBar` 挂几何/工程图 Ribbon。

## 边界确认

| 范围内 | 范围外 |
|--------|--------|
| 宿主顶栏四分段切换 | 左侧 Rail / 底部 Dock |
| `registerWorkspaceMode` + `returnToMainWorkspace` | 改各模式内部业务 |
| softExit 不抢中央 3D | 删除插件顶层菜单 |
| Light/Dark QSS 分段样式 | 新设计系统包 |

## 需求理解

1. 分段始终可见；选中与 `currentWorkspaceMode()` 同步（主程序 = `""`）。
2. 点击驱动已注册 `enterFn` 或宿主 `returnToMainWorkspace`。
3. 插件菜单仍为次要入口，走同一 enter/exit。
4. 修 `claimWorkspaceMode` 对非几何硬清 Ribbon 的逻辑；`setModeToolBar` 互斥显隐。

## 疑问澄清

- 交互形态：已确认方案 2。
- 插件未加载时：仅显示主程序 + 已注册模式（不显示空段）。
