# CONSENSUS — 帮助系统

## 需求描述

在主窗口菜单栏增加 **Help / 帮助**，含：

1. **帮助文档 / Documentation**：打开内嵌帮助浏览器，加载本地 HTML
2. **关于 / About**：`QMessageBox::about` 显示产品简介

## 验收标准

1. 菜单文案随语言切换（Help/帮助、Documentation/帮助文档、About/关于）
2. 点击帮助文档弹出内嵌窗口，显示对应语言 `index.html`
3. HTML 缺失时警告提示，主窗口不崩溃
4. 关于对话框可弹出
5. Debug\|x64 与 Release\|x64 构建通过；`help\` 出现在对应 `OutDir`

## 技术方案

- Widget：`setupMenuBar` + `applyLanguage` + `MainWindowHelp.cpp` + `HelpBrowserDialog`
- 源资源：`CloudSim/help/{zh,en}/index.html`
- 部署：`CloudSim.vcxproj` PostBuild 从 `$(CloudSimRepoRoot)CloudSim\help` 拷到 `$(CloudSimBinDir)help\`
- 路径：`QCoreApplication::applicationDirPath() + "/help/{zh|en}/index.html"`

## 技术约束

- 不引入 QHelp；不改 Host/Core 契约
- 不改工程 `OutDir`；拷贝目标仅为 `$(CloudSimBinDir)help\`
- About 不新建版本号系统

## 任务边界

本期不做完整手册、不插件化 Help、不联网文档。
