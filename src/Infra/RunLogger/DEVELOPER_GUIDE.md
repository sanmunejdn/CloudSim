# RunLogger 模块开发文档

## 1. 模块定位

`RunLogger` 是横切基础设施：为 `CloudSim` 全解决方案提供**统一日志 API**（文件 + 控制台 + 可选 UI 回调）。不依赖 Qt/OSG，可被 `Data`、`RobotUrdf`、`RobotScene`、`Widget` 等任意工程链接。

| 属性 | 说明 |
|------|------|
| 工程类型 | 静态库 / DLL（`RUN_LOGGER_API`） |
| 头文件目录 | `RunLogger/inc/` |
| 实现目录 | `RunLogger/source/` |
| 导出宏 | `run_logger_global.h` → `RUN_LOGGER_API` |

---

## 2. 依赖关系

```mermaid
flowchart LR
  CS[CloudSim.exe] --> RL[RunLogger]
  Widget --> RL
  Data --> RL
  RobotUrdf --> RL
  RobotScene --> RL
```

**被依赖方**：无业务模块依赖；**依赖方**：几乎所有上层模块。

---

## 3. 命名空间 `RunLogger`

### 3.1 `enum class LogLevel`

| 枚举值 | 典型用途 |
|--------|----------|
| `Trace` | 极细粒度跟踪 |
| `Debug` | 开发调试；默认不输出（最低级别为 `Info`） |
| `Info` | 正常流程信息 |
| `Warn` | 可恢复异常 |
| `Error` | 操作失败 |
| `Critical` | 致命错误 |

### 3.2 类型别名

| 名称 | 定义 | 作用 |
|------|------|------|
| `UiSink` | `std::function<void(LogLevel, const std::string&)>` | 将日志行推送到 UI（如 `RunInfoPage`） |

---

## 4. 公共 API（`RunLogger.h`）

### 4.1 生命周期

| 函数 | 签名要点 | 作用 |
|------|----------|------|
| `initialize` | `(logDirectory, fileNameBase = "CloudSim")` → `bool` | 创建日志目录、打开滚动日志文件；应用启动时调用一次 |
| `shutdown` | `void` | 刷盘并关闭 sink；与 `MainWindow::shutdownApplicationLogging()` 配对 |

### 4.2 UI 桥接

| 函数 | 作用 |
|------|------|
| `setUiSink(sink)` | 注册 UI 回调；`MainWindow` 初始化时通常绑定到 `RunInfoPage` |
| `clearUiSink()` | 移除回调（避免析构后悬空调用） |

### 4.3 写日志

| 函数 | 作用 |
|------|------|
| `log(level, message)` | 通用入口：写文件/控制台，并调用 `UiSink`（若已设置） |
| `trace` / `debug` / `info` / `warn` / `error` / `critical` | 按级别的便捷包装 |
| `flush()` | 强制刷盘（`debug`/`info` 默认可能缓冲） |

### 4.4 工具

| 函数 | 作用 |
|------|------|
| `levelName(level)` | 返回级别 C 字符串（用于格式化） |
| `setMinimumLogLevel` / `minimumLogLevel` | 运行时调整最低级别 |
| `isDiagnosticsEnabled()` | 是否输出机器人示教/IK/指令对齐等诊断（见下表） |

### 4.5 调试开关（默认全部关闭）

`main` 在环境变量未设置时写入 `0`：

| 环境变量 | 作用 |
|----------|------|
| `POINTCLOUD_PROCESS_DEBUG` | `1`：最低日志级别降为 `Debug`；并启用 `[示教]`、`[IK残差]`、`[指令显示]`、矩阵自检成功提示 |
| `POINTCLOUD_GIZMO_PIVOT_DIAG` | `1`：`[GizmoPivotDiag]`（`RunLogger::debug`） |
| `ROBOT_KINEMATICS_DEBUG` | `1`/`2`：`[RobotKinematicsDBG]` FK 矩阵（`RunLogger::info`） |

未设置或非 `0`/`false` 时视为开启；空则 `main` 默认 `0`。

---

## 5. 集成约定

1. **启动顺序**：`QApplication` 创建后、`MainWindow` 显示前调用 `RunLogger::initialize`；路径通常为 exe 旁 `logs/`。默认 **不输出** `debug`/`trace`。
2. **退出顺序**：`main` 在 `app.exec()` 返回后调用 `MainWindow::shutdownApplicationLogging()` → 内部 `RunLogger::shutdown()`。
3. **线程**：API 未声明线程安全；**从 UI 线程**调用最安全。后台 `JobSystem` 任务若需打日志，应 `queueOnMainThread` 再写，或仅写文件路径（实现以 `.cpp` 为准）。
4. **环境变量诊断**：专用开关见 §4.5；勿用 `std::cout` 代替 `RunLogger`。

---

## 6. 与架构文档的对应

全局架构见 [`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §4.9。本模块仅提供日志通道，不参与业务状态机。
