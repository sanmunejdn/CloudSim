# FINAL — 帮助系统

## 交付摘要

主窗口增加 **Help / 帮助** 菜单：打开内嵌 HTML 帮助文档，以及 About 对话框。中英多页用户手册（入门、工程、三维、导入、建模、工程图、机器人、轨迹、工艺、点云、标注、PLC/相机、AI、设置、附录）随构建部署到 `bin\x64(d)\help\`。

## 主要改动

| 区域 | 内容 |
|------|------|
| Widget | `HelpBrowserDialog`、`MainWindowHelp.cpp`、菜单与 i18n |
| 资源 | `CloudSim/help/zh|en/index.html` |
| App | `CloudSim.vcxproj` PostBuild 拷贝 help |
| 文档 | `docs/帮助系统/*`、docs/README、Widget DEVELOPER_GUIDE |

## 使用方式

1. 启动 CloudSim
2. 菜单 **帮助 → 帮助文档** → 内嵌窗口显示本地 HTML
3. **帮助 → 关于 CloudSim** → About 对话框
