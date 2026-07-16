# ALIGNMENT：UI 布局与样式修复

## 项目特性

- Qt Widgets 桌面应用（CloudSim）
- 主题：`ApplicationStyle` 全局 QSS（亮/暗）
- 布局：`QMainWindow` + 左/右/底 `QDockWidget`
- 轨迹编辑：`TrajectoryEditPageWidget`（RobotWidget）

## 原始需求

1. 右侧页面与下方日志页重叠，右侧内容显示不全
2. 下拉框视觉过高，展开后无法完整显示项内容
3. 底部按钮组样式无主次区分

## 边界确认

| 在范围内 | 不在范围内 |
|----------|------------|
| Dock 角区布局（日志 vs 侧栏） | 全面 UI 重设计 / 换框架 |
| 全局 QComboBox / QPushButton QSS | 改业务逻辑 |
| 轨迹编辑页按钮角色标注 | 非轨迹页逐页美化（仅全局样式受益） |

## 需求理解

- 截图为「工作区 → 机器人 → 轨迹编辑」；底栏白块为 `RunInfoPage`（运行日志）压住「应用/重置/撤销」等按钮
- 程序/组下拉在 `QGroupBox` 外，未吃到 `kTrajectoryControlHeight=26`，被全局 `QComboBox { padding: 6px 10px }` 撑高
- 全部 `QPushButton` 共用同一灰底样式，无 primary/secondary/danger

## 决策（已拍板）

1. **日志通栏**：`BottomLeft/RightCorner → BottomDockWidgetArea`，侧栏停在日志上方，消除重叠
2. **下拉紧凑**：全局 ComboBox 高度约 26px；弹层项高与滚动明确
3. **按钮角色**：`btnRole=primary|secondary|danger`；应用=主，重置/撤销/重做=次，破坏性=危险

## 疑问澄清

无阻塞歧义；若产品希望「侧栏通高、日志仅中间」可再改角区，但当前重叠症状优先用通栏日志修复。
