# CloudSim 可执行入口开发文档

## 1. 模块定位

`CloudSim` 工程是解决方案的 **bootstrap 层**：仅包含 `main.cpp`，负责进程级初始化后创建并显示 `MainWindow`。业务逻辑在 `Widget.dll` 及其他引擎/功能 DLL 中。

| 属性 | 说明 |
|------|------|
| 输出 | `CloudSim.exe` |
| x64 直接链接 | `Widget.lib`、`Data.lib`（import lib） |
| 运行时依赖 | exe 同目录下全部产品 DLL（见 [`docs/DIRECTORY_LAYOUT.md`](../docs/DIRECTORY_LAYOUT.md)「x64 运行时 DLL」） |
| 源码 | `CloudSim/main.cpp` |

---

## 2. 启动流程

```mermaid
sequenceDiagram
    participant main as main.cpp
    participant Qt as QApplication
    participant MW as MainWindow
    participant RL as RunLogger

    main->>main: applyRobotKinematicsDebugFromArgv
    main->>Qt: QApplication(argc, argv)
    main->>main: configureWindowsDllSearchPath (Win32)
    main->>Qt: setOrganizationName / setApplicationName
    main->>MW: MainWindow w; w.showMaximized()
    main->>Qt: exec()
    main->>MW: shutdownApplicationLogging()
    Note over MW,RL: RunLogger.dll 由 Widget/Data 等加载
```

### 2.1 `applyRobotKinematicsDebugFromArgv`

| 命令行 | 行为 |
|--------|------|
| `--robot-kinematics-debug [0\|1]` | 设置环境变量 `ROBOT_KINEMATICS_DEBUG`，供 `RobotSceneKinematics` 输出 `[RobotKinematicsDBG]` |
| `--robot-kinematics-debug=1` | 同上 |

GUI 程序无控制台时，用此参数代替预先设置系统环境变量。

### 2.2 `configureWindowsDllSearchPath`（仅 Win32）

将以下候选路径** prepend** 到 `PATH`（若目录存在）：

- `<appDir>/../SDK/OSG3.6.5/bin`
- `<appDir>/OSG3.6.5/bin`

确保加载工程自带的 OSG 3.6.5 运行时，优先于系统 PATH 中旧版 `osg161-*.dll`。

### 2.3 Qt 应用元数据

- `QCoreApplication::setOrganizationName("CloudSim")`
- `setApplicationName("CloudSim")`

用于 `QSettings`、主题路径等持久化。

### 2.4 主窗口与退出

```cpp
MainWindow w;
w.showMaximized();
const int code = app.exec();
MainWindow::shutdownApplicationLogging();
return code;
```

`shutdownApplicationLogging()` 内部应调用 `RunLogger::shutdown()`，保证日志文件完整关闭。

---

## 3. 不负责的事项

以下能力**不在**本工程中实现，修改时请转到对应子工程文档：

| 能力 | 文档 |
|------|------|
| UI、文档页、导入/保存 | [`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) |
| 后端数据模型 | [`../Data/DEVELOPER_GUIDE.md`](../Data/DEVELOPER_GUIDE.md) |
| OSG 场景 | [`../OsgWidgetCore/DEVELOPER_GUIDE.md`](../OsgWidgetCore/DEVELOPER_GUIDE.md) |
| 机器人仿真 | [`../RobotScene/DEVELOPER_GUIDE.md`](../RobotScene/DEVELOPER_GUIDE.md) |

---

## 4. 构建注意

- 平台：`Win32` / `x64`，工具集 `v142`，Unicode。
- **x64**：工作目录设为 `bin/x64/`（或 `x64d/`）；exe 旁需有 Qt/OSG/OCCT 及全部产品 DLL（`RunLogger.dll`、`OsgWidgetCore.dll` 等）。
- **Win32**：遗留单体/静态链接路径；`main` 仍可直接链 `RunLogger.lib` 静态库。
- 新增全局启动逻辑（单实例、崩溃转储等）应放在 `main.cpp` 或小型 `AppBootstrap` 单元，避免膨胀 `MainWindow`。

---

## 5. 相关文档

- 总览：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)
- 模块索引：[`../docs/MODULE_DEVELOPER_GUIDES.md`](../docs/MODULE_DEVELOPER_GUIDES.md)
