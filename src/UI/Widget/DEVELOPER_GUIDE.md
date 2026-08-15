# Widget 模块开发文档

> **空间契约**：[`../../../docs/spatial_contract_world_pose.md`](../../../docs/spatial_contract_world_pose.md) §1.1 — `pose`=模型原点世界坐标；Widget 侧属性编辑经 `doc->data().applyPropertyChange`，由 Host `BackendVisualSync` 同步 OSG。

## 1. 模块定位

`Widget` 是 **UI 编排层 DLL**：承载 `MainWindow`、文档页、属性面板、后端树与仿真协调。对应架构中的「UI 层」；不直接持有 OSG 场景核心（在 `OsgWidgetCore`），不直接持有后端数据模型（在 `Data`）。插件宿主实现编入 `CloudSimHost`（见 `CloudSimPluginHost`）。

| 属性 | 说明 |
|------|------|
| 路径 | `src/UI/Widget/` |
| x64 产物 | `bin/x64(d)/Widget.dll`、`Widget.lib` |
| 预处理器 | `WIDGET_LIB`（本 DLL **export**）；消费方无此宏则为 **import** |
| 契约 | [`CloudSimCore/DEVELOPER_GUIDE.md`](../../Contracts/CloudSimCore/DEVELOPER_GUIDE.md) |
| 宿主 | [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) |

**与 `CloudSimHost` 的分工**

| 层 | 模块 | 职责 |
|----|------|------|
| UI 编排 | **`Widget.dll`** | `MainWindow`、树/属性/仿真 Dock、工程 I/O、`DocumentPage` 机器人元数据 |
| 文档宿主 | `CloudSimHost.dll` | 每页 `BackendDataManager` + `OsgWidget`、Core 适配器、`EventHub`、PluginHost |
| 契约 | `CloudSimCore.dll` | `IDataService` / `IRenderView` / `IRobotService` / `EventHub` |

`DocumentPage` **继承** `cloudsim::host::DocumentHost` 并实现 `IRobotSimulationDocument`；新功能优先经 `data()` / `render()` / `events()`，避免 Widget 再直接扩散 OSG/Data 头文件。

**新代码禁区**：勿新增 `#include "BackendDataManager.h"`。`DocumentPage::backend()` / `IPerLinkRobotStateAccessor::backend()` / `BackendSceneDocumentFacade` 为**存量白名单**（运动学写位姿、可见性写回、mesh 几何）；拓扑/跟随/属性优先 `doc->data()`。跟随查询走 `IDataService::followTargetId` / `sceneFacade()`。

**FK 绑定存储**：`HierarchicalRobotInstance` 的 `fkMeshWorldT0` / `outerWorldAtBindByBackendId` / `basePlacementWorld` 为列主序 `core::Mat4`；关节场景句柄仍为 `osg::MatrixTransform*`（per-link 无关节节点时可为空）。

---

## 2. 目录与编译单元

```text
Widget/
├── inc/
│   ├── widget_global.h                    # WIDGET_EXPORT / OSG_WIDGET_API
│   ├── MainWindow.h                       # 主窗口（QMainWindow）
│   ├── MainWindow_p.h                     # 主窗口私有类型（ItemDataRole 等）
│   ├── HelpBrowserDialog.h                # 内嵌 HTML 帮助浏览器
│   ├── DocumentPage.h                     # 单文档页（DocumentHost + IRobotSimulationDocument）
│   ├── ApplicationStyle.h                 # 浅色/深色主题
│   ├── ApplicationSettings.h              # 应用级 UI 偏好持久化
│   ├── RunInfoPage.h                      # 运行日志面板（中央 splitter）
│   ├── MainWindowSelectionState.h         # 选择状态
│   ├── MainWindowSelectionService.h       # 选择服务（静态）
│   ├── MainWindowObjectRepository.h       # 对象仓库（静态）
│   ├── MainWindowImportCaptureRenderController.h  # 导入/捕获/渲染控制器
│   ├── MainWindowSceneInteractionCoordinator.h    # 场景交互协调
│   ├── MainWindowInstructionPropertyUiHost.h      # 指令属性 UI
│   ├── MainWindowRobotHost.h              # 机器人宿主（IRobotDocumentHost 实现）
│   ├── WidgetRenderAccess.h               # 渲染访问辅助
│   ├── WidgetDocumentAccess.h             # 文档访问辅助（插件存量）
│   ├── WidgetOsgViewHost.h                # IRobotOsgViewHost 实现
│   └── WidgetSceneSignalWiring.h          # OsgWidget 信号边界
└── source/
    ├── MainWindow.cpp                     # 核心逻辑、语言切换、选择处理
    ├── MainWindowUiSetup.cpp              # 构造函数、菜单栏、Dock 布局
    ├── MainWindowHelp.cpp                 # 帮助文档 / 关于
    ├── HelpBrowserDialog.cpp              # 内嵌 HTML 帮助浏览器
    ├── MainWindowProjectIo.cpp            # 工程保存/加载
    ├── MainWindowFileImport.cpp           # 模型/点云导入
    ├── MainWindowBackendTree.cpp          # 后端树管理
    ├── MainWindowPlugins.cpp              # 插件加载
    ├── MainWindowRobotHost.cpp            # 机器人宿主集成
    ├── MainWindowSelectionService.cpp     # 选择管理
    ├── MainWindowPropertyPanel.cpp        # 属性面板
    ├── MainWindowAiAssistant.cpp          # AI 助手集成
    ├── MainWindowImportCaptureRenderController.cpp  # 导入/捕获控制器
    ├── MainWindowSceneInteractionCoordinator.cpp    # 场景交互
    ├── MainWindowObjectRepository.cpp     # 对象仓库
    ├── MainWindowInstructionPropertyUiHost.cpp      # 指令属性
    ├── MainWindowRobotStubs.cpp           # 机器人桩函数
    ├── DocumentPage.cpp                   # 文档页实现
    ├── WidgetSceneSignalWiring.cpp        # OsgWidget → MainWindow 信号接线
    ├── WidgetOsgViewHost.cpp              # IRobotOsgViewHost 实现
    ├── ApplicationStyle.cpp               # 主题加载/应用
    ├── ApplicationSettings.cpp            # settings.ini 读写
    └── RunInfoPage.cpp                    # 运行日志
```

**设备页**：`DevicePageWidget` 在 **`RobotWidget`**（[`../RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md) §`DevicePageWidget`），不在本目录；Property Dock「设备」Tab 经 Host 嵌入。单型号资源问题（重复预览图、末端轴）在对应 `resource/models/...` 包内处理，不改通用扫描逻辑。

**编入 `CloudSimHost.vcxproj` 的源码**（路径仍为 `src/UI/Widget/`，勿在 Widget 下维护第二份副本）：

- `OsgWidget.cpp` 及 `OsgWidget*Controller.cpp`、`ObjectTransformOperation.cpp`、`QWidgetViewer.cpp` 等
- `BackendSceneDocumentFacade.cpp`、`BackendFollowReverseIndex.cpp`、`OsgWidgetSceneBridge.cpp`

---

## 3. 架构（本模块内）

```mermaid
flowchart TB
  MW[MainWindow] --> DP[DocumentPage ×N]
  MW --> BT[BackendTree]
  MW --> PP[PropertyPanel]
  MW --> RD[RunInfoPage]
  MW --> SIM[RobotSimulationController]
  MW --> AI[AiAssistantDockWidget]
  MW --> SSW[WidgetSceneSignalWiring]
  MW --> PH[PluginManager<br/>编入 Host]

  DP --> DH[DocumentHost]
  DH --> BDM[BackendDataManager]
  DH --> OW[OsgWidget]
  DH --> DSA[DataServiceAdapter]
  DH --> RSA[RobotServiceAdapter]
  DH --> RVA[OsgRenderViewAdapter]

  SSW --> |信号| OW
  MW --> |契约| DSA
  MW --> |契约| RSA
  MW --> |契约| RVA

  SIM --> RW[RobotWidget]
  AI --> AW[AiWidget]
```

---

## 4. 核心类型

### 4.1 `MainWindow`

`MainWindow` 是应用主窗口，继承 `QMainWindow` 和 `IPluginMainWindowHost`。源码拆分为 15+ 文件实现关注点分离。

| 维度 | 说明 |
|------|------|
| 职责边界 | 菜单栏、Dock 布局、文档标签页、属性面板、后端树、仿真协调、插件加载 |
| 生命周期 | `main.cpp` 构造，`showMaximized()`，进程内单例；关窗/`~MainWindow` 调用 `shutdownRuntimeWorkers`（停仿真定时器 + `JobSystem::shutdown`：清排队、限时 wait，超时弃池以免卡死）再 `pluginManager->shutdownAll` |
| 对外契约 | 通过 `currentPage()` 获取当前 `DocumentPage*`，再经 `data()` / `robot()` / `render()` 访问契约 |
| 事件协作 | 订阅 `EventHub` 事件刷新树/属性面板；`WidgetSceneSignalWiring` 桥接 OsgWidget 信号 |

**源码拆分职责**：

| 文件 | 职责 |
|------|------|
| `MainWindowUiSetup.cpp` | 构造函数、菜单栏、Dock 布局、语言切换初始化 |
| `MainWindowHelp.cpp` | Help 菜单：打开本地 HTML、About |
| 设置 → 模式切换 | 工作区模式入口（主/几何/工艺/工程图） |
| `MainWindow.cpp` | 核心逻辑、`applyLanguage()`、选择处理、`closeDocumentTab()` |
| `MainWindowProjectIo.cpp` | 工程保存/加载（`.pcp` / `.json`） |
| `MainWindowFileImport.cpp` | 模型/点云导入；「插入 → 坐标系」创建 `FrameBackendData` |
| `MainWindowBackendTree.cpp` | 后端树 `QTreeWidget` 管理、右键菜单 |
| `MainWindowPlugins.cpp` | `loadPlugins()` 扫描 `plugins/*/plugin.json` |
| `MainWindowRobotHost.cpp` | `MainWindowRobotHost`（`IRobotDocumentHost` 实现） |
| `MainWindowSelectionService.cpp` | 选择状态管理、gizmo 联动 |
| `MainWindowPropertyPanel.cpp` | 属性面板 `QtTreePropertyBrowser`、防抖提交 |
| `MainWindowAiAssistant.cpp` | AI 助手 Dock 集成 |
| `MainWindowImportCaptureRenderController.cpp` | 导入/捕获/渲染控制器 |
| `MainWindowSceneInteractionCoordinator.cpp` | 场景交互模式切换 |
| `MainWindowObjectRepository.cpp` | 对象仓库管理 |
| `MainWindowInstructionPropertyUiHost.cpp` | 仿真指令属性 UI |
| `MainWindowRobotStubs.cpp` | 机器人桩函数 |

### 4.2 `DocumentPage`

`DocumentPage` 是单文档页，继承 `DocumentHost` 并实现多个接口：

| 接口 | 来源 | 说明 |
|------|------|------|
| `DocumentHost` | `CloudSimHost` | 文档宿主（Data + OSG + Core 适配器） |
| `IRobotSimulationDocument` | `RobotWidget` | 机器人仿真元数据（URDF、关节、FK 绑定） |
| `IRobotUrdfImportContext` | `CloudSimHost` | URDF 导入上下文（场景挂载、后端注册） |
| `IPerLinkKinematicsHost` | `CloudSimHost` | per-link FK 计算委托 |
| `IPerLinkRobotStateAccessor` | `CloudSimHost` | 状态快照提取与结果应用 |

**多实例机器人**：`HierarchicalRobotInstance` 结构体支持多个机器人实例，每个实例独立维护：

- URDF 路径、关节名/限位
- per-link 后端映射（`linkNameToBackendId`）
- FK 绑定矩阵（`fkMeshWorldT0`、`outerWorldAtBind`、`basePlacementWorld` → `core::Mat4`）
- 坐标系（`RobotCoordinateFrameSet`）
- 关节 `MatrixTransform*`（可选；per-link 模式可为空）

**视口工具栏线框**：`ViewportToolBar::wireframeToggled` → `OsgWidget::setWireframeMode`。BRep 走拓扑边（`applyBrepViewportWireframe`），非 BRep 为三角 `PolygonMode::LINE`；与导入 `showWireOutline` 解耦。详见 [`BackendVisual/DEVELOPER_GUIDE.md`](../BackendVisual/DEVELOPER_GUIDE.md) §4.3。

### 4.3 `WidgetSceneSignalWiring`

OsgWidget 信号的**唯一边界**。所有 OsgWidget 的 Qt 信号（拾取、gizmo、拖拽等）在此接线到 MainWindow 槽函数。

| 维度 | 说明 |
|------|------|
| 位置 | `WidgetSceneSignalWiring.cpp` |
| 职责 | 将 OsgWidget 信号转发到 MainWindow 对应处理函数 |
| 设计约束 | MainWindow 不直接 connect OsgWidget 信号；所有接线集中在此文件 |
| 典型信号 | `objectPicked` → `onOsgBackendObjectPicked`；`pointPickFeedback` → `onPointPickFeedback` |

### 4.4 `MainWindowRobotHost`

包含内部类 `DocumentHost`，实现 `IRobotDocumentHost`，将机器人操作委托给 `DocumentPage`。

| API | 委托目标 |
|-----|----------|
| `applyJointAnglesRad` | `DocumentPage::robot().applyJointAnglesRad` |
| `captureToolFrameFromTcp` | `DocumentPage` 坐标系操作 |
| `captureUserFrameFromTcp` | `DocumentPage` 坐标系操作 |
| `resetToolFrame` | `DocumentPage` 坐标系操作 |
| `solveTcpDragTeachIk` | `RobotKinematics::ikPositionDampedLeastSquares` |
| `planForExport` | `RobotPlanInstruction::planMotionInstruction` |

### 4.5 `MainWindowSelectionService`

静态服务，管理选择状态与 gizmo 联动。

| API | 说明 |
|-----|------|
| `selectBackend(MainWindow&, backendId)` | 选中对象、刷新属性面板、设置 gizmo |
| `clearSelection(MainWindow&, keepGizmo)` | 清除选择 |
| `syncSelectionFromOsg(MainWindow&, backendId)` | OSG 拾取 → 选择同步 |

### 4.6 `ApplicationStyle`

浅色/深色主题管理。

| API | 说明 |
|-----|------|
| `applyTheme(QApplication*, Theme)` | 加载 QSS、应用 `CloudSimTreeBranchStyle`、刷新 `UiIcons` |
| `loadSavedTheme()` | 从 `QSettings` 读取 |
| `saveTheme(Theme)` | 写入 `QSettings` |
| `usesDarkTheme()` | 当前是否深色 |

**树展开三角**：全局 QSS 若写过 `QTreeWidget::branch` 会吃掉 Fusion 默认三角，且 Qt5 的 SVG `data:` URI 不可靠。现用 `CloudSimTreeBranchStyle`（`QProxyStyle`）在 `PE_IndicatorBranch` 上绘制 ▸/▾；有子节点才画。Units 树另设 `setRootIsDecorated(true)`。

**按钮角色（QSS 属性）**：`QPushButton` 设置 `btnRole` 为 `primary` / `secondary` / `danger`，主题切换后仍生效。

**运行日志布局**：`RunInfoPage` 挂在中央区竖向 `QSplitter`（文档页上方、日志下方），**不再**使用 `BottomDockWidget`。

**圆角策略**：仅小控件保留 `border-radius`；树/列表/大面积面板强制直角。

### 4.7 `ApplicationSettings`

应用级 UI 偏好持久化，与工程文件（`.pcp`）分离。配置文件路径：**与 `CloudSim.exe` 同目录** 的 `settings.ini`（与 `ai_config.json` 相同约定）。

**与 `ai_config.json` 的分工**：`ai_config.json` 在 exe 旁，管 LLM 端点；`settings.ini` 在用户配置目录，管界面偏好。

**持久化字段**（`ApplicationSettings::UiPreferences`）：

| 分组 | 键 | 说明 |
|------|-----|------|
| `General` | `language` | `zh` / `en` |
| `Appearance` | `theme` | `light` / `dark`（原注册表项首次启动自动迁移） |
| `View` | `leftPanelVisible` / `rightPanelVisible` | 视图菜单「左/右侧面板」勾选 |
| `View` | `leftDockWidth` / `rightDockWidth` | 侧栏宽度（≥160px 才恢复） |
| `SidePanelTabs` | `<objectName>` | 视图菜单各侧栏页签（如 `CloudSim_AiAssistant`、插件 tab） |
| `Workspace` | `modeId` | 设置 → 模式切换 / 顶栏工作区模式 |

**生命周期**

```
启动 → loadUiPreferencesFromStorage()
     → applyLanguage / applyTheme / 侧栏可见性
loadPlugins 完成 → restoreUiPreferencesAfterPlugins()（插件 tab + 工作区模式）
用户改语言/主题/模式/视图勾选 → 即时 persistUiPreferencesToStorage()
关窗 closeEvent / aboutToQuit → persistUiPreferencesToStorage()
loadPlugins 完成 → 先恢复工作区模式，再 applySavedViewLayout()（避免插件模式覆盖侧栏可见性）
```

**侧栏页签键**：优先 `QWidget::objectName()`；插件侧栏应在注册前设稳定 objectName，避免用标题（随语言变化）。

| API | 说明 |
|-----|------|
| `ApplicationSettings::load()` / `save()` | 读写完整偏好 |
| `ApplicationSettings::saveTheme()` / `loadTheme()` | 仅更新主题（避免覆盖其它未保存项） |
| `ApplicationSettings::settingsFilePath()` | 返回 ini 绝对路径（调试/文档） |
| `MainWindow::persistUiPreferencesToStorage()` | 从当前 UI 状态收集并保存 |

---

## 5. 数据流

### 5.1 属性编辑防抖

属性面板使用防抖避免每次 spin 值变化都触发完整重建：

```
用户编辑属性 → onVariantPropertyValueChanged()
  → schedulePropertyPanelCommitRefresh(backendId)
    → m_propertyPanelCommitTimer.start(150ms)
      → onPropertyPanelCommitTimer()
        → flushPropertyPanelVisualCommit(contextId)
          → doc->data().applyPropertyChange(id, key, value)
            → BackendVisualSync::afterDataServicePropertyChange()
```

### 5.2 指令属性刷新

仿真指令属性同样使用防抖：

```
用户选择指令 → onSimulationInstructionSelectionChanged()
  → scheduleInstructionPropertyRefreshDebounced(instruction, refreshFeasibleAxis)
    → m_instructionPropertyRefreshTimer.start(100ms)
      → onInstructionPropertyRefreshTimer()
        → updateInstructionPropertyPanel(instruction, refreshFeasibleAxis)
```

### 5.3 Units 后端对象树（显示框架）

| 约定 | 说明 |
|------|------|
| 文档根 | 每个打开的 `DocumentPage` 一个 top-level 根（标题 = Tab） |
| 对象节点 | 1 对象 1 节点；主父投影（`parentIds.front()`）；无 `(ref)` |
| Annotations | 挂在对应文档根下的「注释」分组 |
| 绑定器 | `BackendUnitsTreeBinder`：`syncDocument` / `showOnlyDocument` / `patchObjectVisible` / 注解增删 |
| 森林 | `BackendUnitsDisplayForest::buildDocument` 由 `listObjectSnapshots()` 投影 |
| 与 OSG 调试树 | Units = 多文档投影；「场景层级」= 仅活动文档 OSG 快照 |

**多文档 Tab**：开工程优先新 Tab（空白未命名可复用）；同路径切已有 Tab。切 Tab 时 stash/restore 该文档的 `ioSignalNetworkCache`。关 Tab 调 `removeDocument`。

过程稿（已归档）：[`docs/_archive/后端对象显示树/`](../../../docs/_archive/后端对象显示树/)。可见性真源：[`docs/_archive/backend_visibility/`](../../../docs/_archive/backend_visibility/)。

### 5.4 后端树批量刷新抑制

层级导入时抑制逐片树刷新：

```cpp
{
    ScopedBackendTreeRefreshSuppress guard(*this);
    // 批量导入...不触发 refreshBackendTree()
}
// guard 析构 → 对受影响文档一次文档作用域 rebuild（迁移后）；现状为一次 refreshBackendTree()
```

---

## 6. 工程 I/O

### 6.1 保存流程

```
onSaveProject()
  → QFileDialog::getSaveFileName()
  → buildProjectSaveRoot(*doc, languageCode, workRoot)
    → 生成 objects[] / edges[] / annotations / camera
  → mergeRobotKinematicsIntoProjectRoot() 已废弃：现由 `RobotProjectIo::writeRobotKinematics` 在 Widget 保存路径直接调用
  → mergeRobotProgramsIntoProjectRoot()
  → mergeRobotProgramsIntoProjectRoot()
  → QJsonDocument → file.write()
  → [可选] project_package_zip::zipDirectoryTree() → .pcp
```

### 6.2 加载流程

```
onOpenProjectFile()
  → 校验 version 后再 clear（`clearContentForProjectOpen`：Data + OSG）
  → 新 Tab 打开（空白未命名可复用；同路径切 Tab）
  → loadProjectObjectsFromJson(objects[])
  → finalizeProjectHierarchyAfterObjects(edges[])
  → restoreRobotKinematicsFromProjectJson()
  → loadRobotProgramsFromProjectJson()
  → finalizeProjectLoadFollowAndViewport()
  → refreshBackendTree()
```

**URDF 空壳根**：`RobotURDF_*` 无三角面；加载须走空壳注册（见 Host §4.2c / §4.4.4），否则树顶只剩 `base_link`。

**显示/隐藏**：树勾选 → `setBackendVisible` / Data 真源 + OSG NodeMask。详见 [`_archive/backend_visibility`](../../../docs/_archive/backend_visibility/)。
---

## 7. 插件集成

### 7.1 插件加载

```cpp
void MainWindow::loadPlugins()
{
    // 扫描 plugins/*/plugin.json
    // QPluginLoader 加载 DLL
    // qobject_cast<ICloudSimPlugin*>(instance)
    // plugin->initialize(hostContext)
    // 插件可注册：side panel tab、dock widget、menu action
}
```

### 7.2 `IPluginMainWindowHost`

Widget 实现此接口，供插件宿主（编入 Host）访问 UI 能力：

| API | 说明 |
|-----|------|
| `currentDocumentHost()` | 当前文档宿主 |
| `documentHostAt(tabIndex)` | 按索引获取文档 |
| `documentTabs()` | 文档标签页控件 |
| `appendRunInfo(message)` | 追加运行日志 |
| `addPluginSidePanelTab(title, widget)` | 添加侧栏标签 |
| `addPluginDockWidget(title, widget, area)` | 添加 Dock |
| `useChinese()` | 当前语言 |
| `menuBar()` / `statusBar()` | 菜单栏/状态栏 |
| `enqueueBackgroundJob(...)` | 后台任务 |
| `mainWindowWidget()` | 主窗口 QWidget* |

**菜单栏结构**（`setupMenuBar`）：File / View / **Insert** / Settings / **Help**。

| 菜单 | 要点 |
|------|------|
| Insert（插入） | `Coordinate Frame…` → `onCreateCoordinateFrame`：对话框填名称+位姿，注册 `FrameBackendData`（`catalogTypeName=CoordinateFrame`）并加载轴可视化 |
| Help（帮助） | `Documentation` → `onOpenHelpDocumentation`：内嵌 `HelpBrowserDialog` 打开 `applicationDirPath()/help/{zh\|en}/index.html`；`About CloudSim` → `QMessageBox::about`。源资源在仓库 `CloudSim/help/`，由 `CloudSim.vcxproj` PostBuild 拷到 `$(CloudSimBinDir)help\` |

---

## 8. 机器人仿真协调

### 8.1 `RobotSimulationController`

仿真控制器（`RobotWidget` 编译），由 MainWindow 持有：

| 能力 | API |
|------|-----|
| 初始化 | `initializePlanners()` |
| 关节角 | `aggregatedJointAnglesRad()` |
| 播放 | `runSimulation()` / `stopSimulation()` |
| 指令 | `addInstruction(type)` / `removeInstruction(id)` |
| 导出 | `exportProgram()` |

### 8.2 `MainWindowRobotHost`

桥接 `MainWindow` ↔ `DocumentPage` 的机器人操作：

```
RobotSimulationController
  → IRobotMainWindowHost (MainWindow 实现)
    → MainWindowRobotHost::DocumentHost (IRobotDocumentHost)
      → DocumentPage::robot() / render()
```

---

## 9. 工程与构建

### 9.1 链接依赖（`Widget.vcxproj`）

`CloudSimUiAssets`（静态）、`CloudSimHost`、`CloudSimCore`、`OsgWidgetCore`、`Data`、`RobotKinematics`、`RobotUrdf`、`RobotScene`、`GeometryEngine`、`RunLogger`、`RobotWidget`、`AiWidget`、`CloudSimAiSDK`、`CloudSimPluginSDK`。

### 9.2 输出与链接路径

[`Directory.Build.props`](../../../Directory.Build.props) 定义：

- Debug：`$(CloudSimBinDir)` → `CGAL5.5.2/bin/x64d/`
- Release：`bin/x64/`

### 9.3 推荐生成顺序

`CloudSimCore` → `Data`（及依赖引擎）→ `CloudSimHost` → **`Widget`** → `CloudSim`。

若仅生成 Widget 报 **LNK1181 找不到 Host.lib**，先单独生成 `CloudSimHost`。

### 9.4 Qt MOC 注意

- `DocumentPage.h`：仅 `<QtMoc Include="inc\DocumentPage.h" />`
- 禁止同一头文件同时出现在 `ClInclude` 与 `QtMoc`

### 9.5 预处理器

| 宏 | 作用 |
|----|------|
| `WIDGET_LIB` | 本 DLL 导出 `WIDGET_EXPORT` |
| `CLOUDSIM_OSG_IN_HOST` | 区分 OSG 符号 import/export（与 Host 工程约定） |

---

## 10. 消费方集成

### 10.1 `CloudSim.exe`

```cpp
#include "CloudSimBootstrap.h"
#include "MainWindow.h"

cloudsimSetApplicationContext(cloudsimCreateApplicationContext());
MainWindow w(cloudsimApplicationContext()->events());
w.showMaximized();
```

### 10.2 插件

插件通过 `IPluginMainWindowHost` 接口访问 Widget 能力，不直接 include Widget 头文件。

### 10.3 新代码应遵守

1. **跨模块数据**：经 `CoreTypes` DTO 与 `IDataService` / `IRenderView`，禁止 `void*` 或 JSON 穿透。
2. **OSG 修改**：改 `Widget/source/OsgWidget*.cpp`（由 Host 编译）；核心场景改 `OsgWidgetCore`。
3. **事件**：向 `EventHub` 发布/订阅（`CoreEvents.h`）；选择/姿态刷新优先走 `SelectionChanged` / `PoseCommitted`。
4. **信号接线**：OsgWidget 信号统一在 `WidgetSceneSignalWiring` 接线，禁止 MainWindow 直接 connect OsgWidget。
5. **属性编辑**：使用防抖定时器（`m_propertyPanelCommitTimer`），避免每次 spin 触发完整重建。
6. **树刷新**：批量操作使用 `ScopedBackendTreeRefreshSuppress` RAII 守卫。

---

## 11. 网页端（CloudSimWeb）坐标系说明

网页 UI 在 `CloudSim/web/cloudsim-web-ui/public-fallback`，经 `CloudSimWebGateway` 调 Host。勿混淆两套「坐标系」：

| 概念 | 用途 |
|------|------|
| **FrameBackendData**（catalog `CoordinateFrame`） | 场景独立坐标系；菜单「插入 → 坐标系」创建；轨迹算子「转换工件型」外部 TCP |
| **RobotCoordinateFrameSet**（机器人 Tab「坐标系」） | 工具系/用户系；与场景 Frame **正交**，不互相替代 |

对齐桌面的关键路径：

1. **创建**：`POST /api/objects/coordinate-frame` → `registerAdoptedFrameAndLoadScene`（同 `MainWindow::onCreateCoordinateFrame`）。列表：`GET /api/objects/coordinate-frames`，或从 `GET /api/objects` 按 `className=FrameBackendData` 过滤。
2. **转换工件运行时**：`HeadlessTrajectorySession::runPipelineOnWorldRaw` 在 `executeFull` 前注入当前机器人 TCP（`setWorkpieceReferenceInBase`）与 Frame 解析器（`setExternalTcpFrameResolver`），对齐桌面 `TrajectoryEditSession::injectWorkpieceReferenceOnEngine`。无 TCP 时算子报「缺少当前机器人 TCP 参考位姿」。
3. **前端**：插入对话框创建 Frame 并在 Three.js 中画 RGB 短轴；轨迹 op 表单对 `toWorkpiece.externalTcpBackendId` 渲染 Frame 下拉，选中后隐藏手动六自由度。

构建：对 `CloudSimHost` → `CloudSimWebGateway` → `CloudSimWeb` 编 **Debug\|x64** 与 **Release\|x64**；静态资源落到 `bin\x64d\web` / `bin\x64\web`。

---

## 12. 与相关文档

| 文档 | 内容 |
|------|------|
| [文档索引](../../../docs/README.md) | 全局架构与依赖图 |
| [`CloudSimHost/DEVELOPER_GUIDE.md`](../../Host/CloudSimHost/DEVELOPER_GUIDE.md) | 文档宿主、Core 适配器、组合根 |
| [`CloudSimCore/DEVELOPER_GUIDE.md`](../../Contracts/CloudSimCore/DEVELOPER_GUIDE.md) | 契约接口与 DTO |
| [`RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md) | 仿真 UI、轨迹编辑 |
| [`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../CloudSimPluginHost/DEVELOPER_GUIDE.md) | 插件宿主（**编入 `CloudSimHost.dll`**） |
| [`OsgWidgetCore/DEVELOPER_GUIDE.md`](../OsgWidgetCore/DEVELOPER_GUIDE.md) | 场景核心、gizmo、拾取、示教罗盘几何 |

### 13.1 TCP 拖动示教（OSG 侧，与 RobotWidget 协作）

编排在 `RobotSimulationController`；OSG 在 `OsgWidgetTcpTeach` / `RobotTcpDragTeachOperation`。

| 要点 | 约定 |
|------|------|
| 进入示教 | FK 目标 + 法兰挂载 `toolLocalOnFlange`；`reconcilePerLinkOuterBindFromScene` 后 `beginTcpDragTeach` |
| 罗盘位姿 | overlay 跟 `T_base_target × P`；法兰路径在 `syncTargetInBase` 时用挂载点世界位姿反推 `T_base`，避免 P 与外绑不一致时罗盘落在默认位 |
| 重绘 | 示教激活期间 `requestRedraw` 走 `syncTcpTeachWorldPatFromTarget`（勿在 IK 追赶时切 `FromMount`，否则 Local 轴闪跳） |
| 轨迹叠加 | `clearImportedContent` 后须把 `m_tcpTeachSceneOverlayGroup` 挂回 `m_trajectoryOverlayGroup`，否则罗盘游离不可见 |

详见 [`RobotWidget/DEVELOPER_GUIDE.md`](../RobotWidget/DEVELOPER_GUIDE.md)「TCP 拖动 IK」。

---

## 13. 演进路线

**已完成**

1. ~~**属性 `IDataService`**~~：`MainWindowPropertyPanel` 普通行走 `doc->data()`。
2. ~~**EventHub 选择/姿态**~~：`publishSelectionChanged` / `publishPoseCommitted`；`MainWindowUiSetup` 订阅刷新属性面板。
3. ~~**工程 I/O 收口**~~：`buildProjectSaveRoot` / `loadProjectObjectsFromJson` 经 Host 集中。
4. ~~**PluginHost 迁入 Host**~~：`CloudSimPluginHost` 源码编入 `CloudSimHost.vcxproj`。
5. ~~**Robot 收口**~~：`IRobotDocumentHost` 委托 Host 实现；per-link 通过 `IPerLinkKinematicsHost` 收口。

**仍待 / 长期**

1. **OSG 头文件解耦**：`DocumentPage` 等仍保留 Robot* 头用于访问器实现（未来可通过 Pimpl 隔离）。
2. **`IRenderView` 全面替代**：Widget 主路径走 `render()`；阶段 3.3-3.4（`ObjectTransformOperation` 等）待定。
3. **`RobotSimulationController` 核心逻辑迁入 Host**：仿真编排逻辑下沉（长期规划）。
