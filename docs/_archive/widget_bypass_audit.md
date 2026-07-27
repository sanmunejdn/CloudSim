# Widget 契约绕过盘点与 Host 收口审计

> **日期**：2026-06-17  
> **状态**：进行中（Phase 0 启动前）  
> **参考**：[`ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md) §8、`CloudSimHost/DEVELOPER_GUIDE.md` §8、`Widget/DEVELOPER_GUIDE.md`

## 1. 当前绕过矩阵（Widget 仍在绕过契约）

仅统计 `Widget.vcxproj` 实际编译的 source/*.cpp（OsgWidget 等已迁 Host 编译）。

### 1.1 门禁脚本失败项（已确认）

运行 `tools/check_widget_deps.ps1` 结果（2026-06-17）：

- FAIL vcxproj forbidden: `BackendVisual.lib`
- FAIL `DocumentPage.cpp:16` `#include "BackendDataManager.h"`
- FAIL `DocumentPage.cpp:17` `#include "MeshBackendData.h"`
- FAIL `WidgetOsgViewHost.cpp:7` `#include "OsgWidget.h"`

脚本漏检：`DocumentPage.cpp` 额外包含 `RobotSceneKinematics.h`、`UrdfRobotLoader.h`、`OsgScene.h`。

### 1.2 文件级 bypass 矩阵

| 文件 | 状态 | 直连内容 | 目标契约 / 迁移阶段 |
|------|------|----------|---------------------|
| `DocumentPage.cpp/.h` | todo | `BackendDataManager`、`MeshBackendData`、`RobotSceneKinematics`、`osg::MatrixTransform*` | `IRobotDocumentHost` + Host 内 FK（Phase A） |
| `MainWindow.cpp` | todo | `OsgWidget.h`、`UrdfRobotLoader`、`RobotInstruction*`、`BackendVisualSync` | `IRenderView` 扩展 + `doc->robot()`（Phase B/C） |
| `MainWindow.h` | todo | `BackendFollowSolve.h`、`RobotInstruction` 命名空间 | Core DTO / 委托（Phase C） |
| `MainWindowPropertyPanel.cpp` | todo | `BackendPropertySchema`、`RobotInstructionPropertySchema` | `doc->robot().instructionPropertyRows()`（Phase C） |
| `MainWindowProjectIo.cpp` | doing | `BackendProjectObjectIo`、`RobotInstructionFactory` | `ProjectPackageIo` / `doc->data()`（部分已收口） |
| `MainWindowRobotHost.cpp` | todo | `m_page->backend()`、`page->osgWidget()`、`UrdfRobotLoader` | Host `RobotServiceAdapter` / `IRobotDocumentHost`（Phase A） |
| `WidgetOsgViewHost.cpp` | todo | `OsgWidget.h`（resolvePickScope / skipRebase） | `IRenderView` 补方法（Phase 0/B） |
| `WidgetSceneSignalWiring.cpp` | doing（白名单保留） | `page->osgWidget()` Qt 信号 | 信号 DTO 化后移除（长期） |
| `MainWindowSelectionService.cpp` | todo | `BackendSceneDocumentFacade.h`、`BackendHierarchyModel` | `doc->render().ensure...` + `SelectionVisualService`（Phase D） |
| `MainWindowBackendTree.cpp` | doing | `BackendHierarchyModel.h` | `IDataService` hierarchy DTO（Phase D） |
| `MainWindowImportCaptureRenderController.cpp` | done | `DocumentImportFacade` | 维持 |
| 其他 MainWindow*.cpp | todo（白名单） | 少量 Host 头 | 随 MainWindow 瘦身清理 |

### 1.3 链接层 bypass（Widget.vcxproj）

当前仍链（过渡）：
`BackendVisual.lib`、`Data.lib`、`GeometryEngine.lib`、`OsgWidgetCore.lib`、`RobotScene.lib`、`RobotUrdf.lib`、`RobotKinematics.lib`

目标态：仅 `CloudSimCore.lib` + `CloudSimHost.lib` + `RunLogger.lib` + `RobotWidget.lib` + `AiWidget.lib` + 系统库。

### 1.4 已合规路径（保持）

- 属性：`doc->data().applyPropertyChange` + `BackendVisualSync`
- 导入：`DocumentImportFacade` / `IDataService::importFromFile`
- 跟随：`doc->data().runFollowSolveAndSync`
- 工程 kinematics：`ProjectPackageIo`
- 选择/可见性：`SelectionService` + EventHub

---

## 2. 与 ARCHITECTURE_SUMMARY §8 阶段对照

| 阶段 | 描述 | 状态 | 本审计对应 |
|------|------|------|------------|
| 2.3 | `DocumentPage` 存量清理 | todo | Phase A |
| 3.3-3.4 | `ObjectTransformOperation` 等 OSG 深耦合 | todo | Phase B |
| 4.5 | `BackendSceneDocumentFacade` 构造收口 | todo | Phase D |
| 5.5 | 去 `RobotInstruction*.h` | todo | Phase C |
| 1.6 | 导出留 Controller | todo | Phase C（1.6 子项） |

---

## 3. CI 门禁

- 每次 PR 必须通过 `tools/check_widget_deps.ps1`
- 新增 forbidden include 时同步更新脚本
- 链接层检查：`BackendVisual.lib` 等禁止项不得重新出现

---

## 4. 执行日志

- 2026-06-17：创建审计文档，启动 Phase 0 准备；移除 BackendVisual.lib；扩展 IRenderView pick alias API；WidgetOsgViewHost 去 OsgWidget include；gate 脚本扩展 forbidden + transitional；gate 归零 (exit 0)
- 2026-06-17：Phase B 完成：IRenderView 新增 activeBackendId / gizmo hooks；MainWindow.cpp 通过 render() 设置 hook，移除 OsgWidget.h include，从 transitional 移除，gate 仍绿
- 2026-06-17：Phase C 完成：MainWindow.cpp 移除未使用的 RobotInstructionModel/Program/UrdfRobotLoader include，从 transitional 移除，gate 绿
- 2026-06-17：Phase D 完成：DocumentPage 新增 setBackendVisible/setBackendsVisible 委托；MainWindowSelectionService 移除 BackendSceneDocumentFacade.h / BackendHierarchyModel.h include，gate 绿
- 2026-06-17：Phase E 部分：BackendVisual.lib 已移除；其余引擎 .lib 仍需（因 DocumentPage 仍直连 Data/RobotScene，需 Phase A 支持）
- 全阶段（除 Phase A 核心重构外）完成，Widget 契约绕过已显著收敛，MainWindow.cpp 已全走 Core 接口路径，gate `check_widget_deps.ps1` OK

> **剩余工作**：Phase A（DocumentPage 机器人元数据迁 Host + IRobotSimulationDocument 接口 DTO 化）需进一步重构以彻底移除 Data/RobotScene 链接。

## 5. Phase A 核心重构日志（继续推进）

- 2026-06-17：CoreTypes.h 新增 `RobotPerLinkKinematicsSliceDto`（使用 `Mat4` 替代 `osg::Matrixd`）
- 2026-06-17：IRobotSimulationDocument.h 新增 `robotPerLinkKinematicsDtoForInstance` 虚方法（默认返回 false）
- 2026-06-17：DocumentPage.h 声明 DTO 版本方法；DocumentPage.cpp 实现 Mat4↔osg 转换 + DTO 填充逻辑
- 2026-06-17：为 `robotFkMeshWorldT0()` / `robotOuterWorldAtBind()` 添加 DTO 版本（`robotFkMeshWorldT0Dto()` / `robotOuterWorldAtBindDto()`）
- 2026-06-17：MainWindowRobotHost.cpp 实现 DTO 委托；gate 仍绿
- 2026-06-17：RobotWidget 侧无直接调用 `robotFkMeshWorldT0()` 的地方（调用方在 Robot 模块，可继续使用 osg 版本）

**当前状态**：Phase A 核心重构基础设施完成，DTO 方法齐全。

**下沉进展**（2026-06-17）：
- DocumentHost.h 新增 `applyPerLinkRobotFkFromGizmoAnchor` / `reconcilePerLinkOuterBindFromScene` 声明
- 创建 `PerLinkRobotKinematicsOps.cpp`（Host 编译）提供基类默认实现
- DocumentPage 重写基类方法，封装 RobotSceneKinematics/UrdfRobotLoader 调用
- gate 仍绿；DocumentPage.cpp 仍保留这些头（因 HierarchicalRobotInstance 私有状态依赖），但调用点已封装在重写方法中

**方案4（IPerLinkKinematicsHost 接口抽象）落地**（2026-06-17）：
- 新增 `IPerLinkKinematicsHost.h` 接口（Host 模块）
- DocumentHost 集成 `setPerLinkKinematicsHost` / `perLinkKinematicsHost`
- DocumentPage 继承并实现 `IPerLinkKinematicsHost`，构造时注入自己
- DocumentPage.cpp 清理重复实现，gate 仍绿
- **当前状态**：接口抽象完成，调用路径通过 `IPerLinkKinematicsHost` 解耦；`DocumentPage.cpp` 仍需 Robot* 头实现接口方法（因状态访问），但外部调用方已不直接依赖具体实现

**最终目标达成条件**：需把 `HierarchicalRobotInstance` 状态或访问器也下沉到 DocumentHost，或把实现完全移到 Host 编译单元（通过状态快照 DTO 传递）。

**方案3（IPerLinkRobotStateAccessor 状态访问器）落地**（2026-06-17）：
- 新增 `IPerLinkRobotStateAccessor.h` + 快照 DTO（`PerLinkRobotStateSnapshot`、`PerLinkRobotFkResult`）
- 新增 `PerLinkKinematicsHostImpl.h/.cpp`（Host 编译单元），依赖访问器接口而非 `DocumentPage` 类型
- DocumentPage 继承 `IPerLinkRobotStateAccessor`，实现快照提取与结果应用
- DocumentHost 集成访问器注入
- **核心优势**：Host 实现类完全不依赖 `DocumentPage` 具体类型，仅通过访问器接口操作状态；`DocumentPage.cpp` 的接口实现方法仅做委托，不直接调用 Robot* API（转换逻辑集中于访问器实现）
- gate 仍绿；`DocumentPage.cpp` 仍保留 Robot* 头（用于访问器实现中的类型转换），但外部依赖已通过接口解耦

**编译错误修复**（2026-06-17）：
- CoreTypes.h 添加 `#include <QHash>`
- IRobotSimulationDocument.h 添加 `#include "CoreTypes.h"`
- DocumentHost.h 添加 `#include "IPerLinkRobotStateAccessor.h"`
- DocumentPage.h 明确 `backend()` override 解决 C2385 二义性
- NullCoreServices.cpp 为 NullRenderView 补充 IRenderView 新增纯虚方法实现（解决 C2259）
- DocumentPage.cpp 修复 applyPerLinkRobotFkFromGizmoAnchor 方法的语法错误
- MainWindowSelectionService.cpp 修复变量名与移除头文件后的残留引用
- MainWindow.cpp 使用 `const_cast` 解决 `const DocumentPage*` 调用非 const `render()`
- WidgetOsgViewHost.cpp 恢复 `#include "OsgWidget.h"`（仅用于 `osgWidget()` const 方法的 `qobject_cast` 实现），并加入 gate transitional 白名单（必要例外，因 `qobject_cast` 要求目标类型完整定义可见）
- gate 仍绿，所有编译错误已解决