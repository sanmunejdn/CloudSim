# ACCEPTANCE — 帮助系统

## 验收检查

| 项 | 结果 |
|----|------|
| Help 菜单含「帮助文档」「关于」 | 通过（`MainWindowUiSetup`） |
| 中英 i18n | 通过（`applyLanguage`） |
| 内嵌窗口打开 HTML | 通过（`HelpBrowserDialog` + `MainWindowHelp`） |
| 缺文件警告 | 通过（`QFileInfo::exists` 分支） |
| Debug\|x64 编译 | 通过（Widget + CloudSim） |
| Release\|x64 编译 | 通过（Widget + CloudSim） |
| `bin\x64d\help\{zh,en}\index.html` | 通过（多页手册 + styles.css） |
| `bin\x64\help\{zh,en}\index.html` | 通过 |

## 手册覆盖（用户功能）

入门界面、工程 IO、三维拾取、导入、场景属性、几何建模、工程图、机器人仿真、轨迹、工艺流程、点云网格、几何插件、标注/PointNet、PLC/相机、AI、设置、附录。
