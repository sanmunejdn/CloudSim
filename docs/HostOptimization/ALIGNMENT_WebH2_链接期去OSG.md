# ALIGNMENT — WebH2 链接期去 OSG

## 原始需求

Web 进程链接期不依赖 `OsgWidgetCore`（运行期已 headless）。见 `docs/_archive/网页端/TODO_网页端功能对等.md` §H2。

## 边界

| 做 | 不做（本轮） |
|----|--------------|
| 新增 `CloudSimHostHeadless` 工程，产物 `CloudSimHostHeadless.dll` | 拆 PluginHost/Ai 独立 DLL |
| 排除 Widget OsgWidget* 编译单元与 `OsgWidgetCore` 链接 | 本轮不强求去掉 `RobotUrdf`→BackendVisual/OSG 传递（记 Phase2） |
| `CloudSimWeb.sln` / Web.exe / Gateway 改链 Headless Host | 从 sln 删项目却仍让完整 Host ProjectReference OSG（会掉 Win32/x86d） |
| Debug\|x64 + Release\|x64 验证 | 改桌面 `CloudSimHost` 行为 |

## 验收

- `dumpbin /dependents CloudSimHostHeadless.dll` 无 `OsgWidgetCore.dll`
- `CloudSimWeb.exe` 链 `CloudSimHostHeadless.lib`，不直链 `osg*.lib`（尽量）
- 桌面 `CloudSimHost` 仍编 OSG，双配置通过
