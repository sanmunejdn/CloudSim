# PlcCommUI

Qt 5.14 前端 DLL，通过 **PlcCommSDK** 访问 PLC；**不**链接 CloudSim 的 Widget/Data/PluginHost，**不**包含 `libplctag.h`。

## 入口

```cpp
#include "PlcCommWidget.h"

QWidget* panel = createPlcCommWidget(parent);
```

## 架构

| 类 | 职责 |
|----|------|
| `PlcCommWidget` | 连接表单、标签表、十六进制读写、轮询、日志 |
| `PlcCommController` | 持有 `QThread`，将调用排队到 Worker |
| `PlcCommWorker` | 唯一持有 `IPlcCommClient` 的线程对象 |

## 构建

- 工程：`PlcCommUI.vcxproj`
- 依赖：`PlcCommSDK`（ProjectReference）
- 输出：`bin/x64d/PlcCommUI.dll`（Debug）或 `bin/x64/`（Release）
- 运行时需要同目录的 `PlcCommSDK.dll` 与 `plctag.dll`

## 嵌入宿主

宿主进程加载 `PlcCommUI.dll` 后调用 `createPlcCommWidget` 即可；无需注册 CloudSim 插件 manifest。

## 中英文

- `setUseChinese(bool)`：默认 `true`（中文）
- `applyLanguage()`：刷新控件文案
- CloudSim 插件侧：初始化读 `IPluginHostContext::useChinese()`，并注册 `onLanguageChanged`（见 `PlcCommPlugin`）
