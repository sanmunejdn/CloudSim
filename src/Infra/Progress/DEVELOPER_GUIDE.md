# Progress 模块开发文档

> **文档导航**：[全库入口](../../docs/README.md) · [目录布局](../../docs/DIRECTORY_LAYOUT.md)

## 1. 模块定位

`Progress` 是横切基础设施：为全解决方案提供**统一进度 API**（可选 UI 回调）。不依赖 Qt，可被任意工程链接；桌面由 `Widget` 的 `ProgressDialogController` 经 `setUiSink` 弹出通用进度对话框。

| 属性 | 说明 |
|------|------|
| 工程类型 | x64：动态库 `Progress.dll`；Win32：静态 `.lib` |
| 头文件 | `Progress/inc/` |
| 导出宏 | `progress_global.h` → `PROGRESS_API` |
| 构建定义 | x64：`PROGRESS_LIB`；Win32：`PROGRESS_STATIC` |

## 2. 约定

- `fraction ∈ [0,1]`；`< 0` 表示不确定进度
- 误用 `>1.5`（如 0–100）时内部按 `/100` 归一化，仍应在调用方显式改为 `[0,1]`
- 取消为协作式：对话框触发 `JobSystem::cancel`，worker 须查 `JobCancelToken`
- 无 sink 时 `begin/report/end` 为 no-op

## 3. 与 JobSystem

桌面 `JobSystem` 在 `reportJobStarted/Progress/Finished` 时同步调用 `Progress::begin/report/end`（同一 `jobId`），使插件 `enqueueJob` 自动出条。算法层可继续只用 `JobProgressSink`，不必直接依赖本 DLL。

## 4. 用法示例

```cpp
#include "Progress.h"

Progress::Scope scope("Import model", /*cancellable=*/false);
scope.report(0.2, "Reading...");
scope.report(0.8, "Meshing...");
// 析构自动 end(true)
```
