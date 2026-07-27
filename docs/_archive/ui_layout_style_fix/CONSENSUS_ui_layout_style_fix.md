# CONSENSUS：UI 布局与样式修复

## 需求与验收

| ID | 验收标准 |
|----|----------|
| A1 | 最大化窗口下，右侧轨迹编辑底栏按钮完整可见，不被日志遮挡 |
| A2 | 拖高日志 Dock 后，右侧内容区变矮并可滚动，仍不发生图层重叠 |
| B1 | 程序/组/工艺模板下拉闭合高度约 24–28px，文字垂直居中无大块空白 |
| B2 | 下拉展开后可见全部选项（不足则滚动），长文本不无故裁切到不可读 |
| C1 | 「应用」为蓝色主按钮；重置/撤销/重做为次要样式；视觉可区分 |

## 技术方案

1. `MainWindowUiSetup::setupDockWidgets`：`setCorner` 底角归日志；必要时限制日志默认高度
2. `ApplicationStyle.cpp`：收紧 QComboBox；增加 `QPushButton[btnRole=...]`；亮暗双主题同步
3. `TrajectoryEditPageWidget`：程序/组 Combo 固定高度；按钮 `setProperty("btnRole", ...)`

## 约束

- 不改 Dock 信息架构与功能
- 样式走现有 QSS，不用新第三方库
- RobotWidget 不反向依赖 Widget 新 API（仅用 QSS 属性）

## 假设（已确认）

- 日志通栏优于侧栏通高
- 主色沿用现有 `#0066cc` / `#288cf0`
