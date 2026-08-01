# ALIGNMENT — 帮助系统

## 项目与任务特性

| 项 | 说明 |
|----|------|
| 宿主 | CloudSim 桌面应用（Qt 5.14.2 + Widget 主窗口） |
| 入口 | 菜单栏 `Help` / `帮助` |
| 文档形态 | 本地 HTML，内嵌 `QDialog` + `QTextBrowser` |
| 本期范围 | 打通菜单 → 打开帮助；About；最小示例 HTML |

## 原始需求

用户点击菜单栏帮助按钮，能够打开软件的帮助文档。

## 边界确认

| 在范围内 | 不在范围内 |
|----------|------------|
| Help 菜单：帮助文档 + 关于 | Qt Assistant / `.qch` |
| 内嵌窗口打开 `help/zh|en/index.html` | 系统默认浏览器打开 |
| 构建拷贝到 `$(CloudSimBinDir)help\` | 完整用户手册撰写 |
| 中英菜单与按语言选 HTML | 在线文档 / 搜索引擎 |

## 需求理解

- 菜单在 `MainWindow::setupMenuBar`，与 File/View/Insert/Settings 并列，Help 置末。
- 资源按 `applicationDirPath()/help/{zh|en}/index.html` 解析，与现有运行时路径约定一致。
- 工程文档（`docs/`、`DEVELOPER_GUIDE`）不是产品帮助正文。

## 疑问澄清（已决策）

1. 打开方式 → **内嵌窗口（1A）**
2. 内容范围 → **帮助文档 + 关于；HTML 最小示例（2C）**
