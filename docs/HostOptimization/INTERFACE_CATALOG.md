# INTERFACE_CATALOG — CloudSimHost

> Wave1 卫生文档。模块：`src/Host/CloudSimHost/` → `CloudSimHost.dll`。  
> 真源不仅 `inc/`：`CloudSimHost.vcxproj` 还编入 `src/UI/Widget/`（OsgWidget 壳）与 `src/UI/CloudSimPluginHost/`（Plugin + AI）。  
> 对照：[`DEVELOPER_GUIDE.md`](../../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) §1–§4、§7。  
> 分类：**Stable** = 跨模块长期入口；**Internal** = Host 内/过渡面，新调用勿扩散。

| 调用方缩写 | 含义 |
|------------|------|
| Widget | `Widget` / `DocumentPage` / `MainWindow*` |
| Gateway | `CloudSimWebGateway` / Web REST |
| Plugin | 动态插件（经 PluginSDK；Host 内 Adapter 实现） |
| AI | `CloudSimPluginHost/Ai/*`（编入 Host） |
| Internal | Host 适配器 / 同 DLL 其它 CU |

---

## Stable ABI whitelist

以下为**推荐对外稳定面**（白名单只减不增；新能力优先进 `CloudSimCore`）：

| 入口 | 说明 |
|------|------|
| `CloudSimHost.h` | `createDocumentHost` / `createHeadlessDocumentHost` / `createHostRenderViewFactory` / `documentHostFromScope`；C ABI `cloudsimCreateRenderViewFactory` |
| Bootstrap（声明在 `CloudSimBootstrap.h`，实现于 Host） | `cloudsimCreateApplicationContext` / `cloudsimCreateHeadlessApplicationContext` / `cloudsimSetApplicationContext` / `cloudsimApplicationContext` |
| Core via `DocumentHost` | `data()` → `IDataService`；`robot()` → `IRobotService`；`render()` → `IRenderView`；`events()` → `EventHub` |
| Headless*（Gateway） | `HeadlessRobotContext` / `HeadlessTrajectorySession` / `HeadlessPointCloudBridge` / `HeadlessInstructionPropertyDelegate`；经 `createHeadlessDocumentHost` 持有 |
| Desktop 插件加载 | `PluginManager`（`CLOUDSIM_HOST_EXPORT`）；插件业务面仍走 PluginSDK（`IPluginHostContext` 等），**不**链 Host 头 |

**非白名单但常用过渡面**：`DocumentImportFacade`、`ProjectPackageIo`、`DocumentHostEvents`、`backend()`（白名单只减不增）、`DocumentHostAccess::osgWidgetFrom`（Host 内部）。

---

## A — Composition（组合根 / 工厂）

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/CloudSimHost.h` | `createDocumentHost`, `createHeadlessDocumentHost`, `createHostRenderViewFactory`, `documentHostFromScope`, `cloudsimCreateRenderViewFactory` | Widget / App / Gateway / Internal | **Stable** |
| `inc/DocumentHost.h` | `DocumentHost`；`data`/`robot`/`render`/`events`；`osgWidget`；`backend`；`sceneFacade`/`sceneBridge`/`followReverseIndex`；Follow API；`headless*`；per-link / instruction 注入 | Widget（`DocumentPage` 继承）；Gateway（headless）；Internal；PluginHost | **Stable**（Core 三件套）；`backend()` → **Internal** |
| `inc/DocumentProjectSidecar.h` | 工程旁路表 `sourcePath`/`sourceType`/`parentId`/`projectFilePath` | `DocumentHost` | **Internal** |
| `inc/DocumentFollowState.h` | Follow 脏集、forced、suppress、defer 视觉全量同步 | `DocumentHost` | **Internal** |
| `inc/HostRenderViewFactory.h` | `HostRenderViewFactory::createView`, `wrapOsgWidgetAsRenderView` | Bootstrap / App / 动态加载方 | **Stable** |
| `inc/DocumentHostAccess.h` | `osgWidgetFrom(DocumentHost&)` | Internal（Host CU）；勿经 `render().widget()` 构造期 | **Internal** |
| `inc/cloudsim_host_global.h` | `CLOUDSIM_HOST_EXPORT` / `CLOUDSIM_HOST_LIB` | 全模块 | **Stable**（宏约定） |
| `inc/widget_global.h` | Host 编 OSG 时 `WIDGET_EXPORT` / `OSG_WIDGET_API` → export | OsgWidget 编译单元 | **Internal**（构建约定） |
| `../../App/CloudSimBootstrap/inc/CloudSimBootstrap.h` | `cloudsimCreateApplicationContext`, `cloudsimCreateHeadlessApplicationContext`, `cloudsimSet/ApplicationContext` | `CloudSim.exe` / Web 进程 `main` | **Stable**（实现于 Host） |

**DEVELOPER_GUIDE 锚点**：§4.1 `DocumentHost`，§4.5 `ApplicationContextImpl`，§4.6 `HostRenderViewFactory`，§4.8 C 导出。

---

## B — Core adapters

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/adapters/DataServiceAdapter.h` | `DataServiceAdapter` → `IDataService`（注册/属性/导入/拓扑/`runFollowSolveAndSync`/`followTargetId`…） | Widget / Plugin / AI / Gateway 经 `doc->data()` | **Stable**（契约实现；头本身偏 Internal） |
| `inc/adapters/OsgRenderViewAdapter.h` | `OsgRenderViewAdapter` → `IRenderView`（矩阵/拾取/相机/标注/selection visual…） | Widget / RobotWidget（经 `render()`）；Headless 路径可为 Null | **Stable**（契约实现） |
| `inc/adapters/RobotServiceAdapter.h` | `RobotServiceAdapter` → `IRobotService`（`registerUrdfRobot`/`applyJointAnglesRad`/`planInstruction`/程序 JSON/指令属性） | Widget / Gateway / RobotWidget 经 `doc->robot()` | **Stable**（契约实现） |
| `inc/SelectionVisualService.h` | `SelectionVisualService::ensureSelectionVisual` | Internal / `DocumentHost` / Adapter | **Internal** |

**DEVELOPER_GUIDE 锚点**：§4.2–§4.4。

---

## C — Import（文件 / 层级 / 采纳注册）

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/DocumentImportFacade.h` | `importFileIntoDocument`, `ImportFileResult`, `registerAdoptedMesh`/`registerAdoptedPointCloud`, `PointCloudBackgroundLoadState`, `ModelBackgroundLoadState` | Widget 导入菜单/Job；Plugin `importFileIntoActiveDocument`；AI；Gateway 可共路 | **Stable**（Host 公共导入面） |
| `inc/BackendFileImport.h` | `importMeshFile`, `importPointCloudFile`, `registerAdopted*AndLoadScene`, `registerAdoptedBackendObject`, `rekeyBackendObject`, `attachBackendChildToCustomDevice` | Internal（Facade/工程 IO）；少量 Widget | **Internal**（优先走 Facade） |
| `inc/HierarchyMeshImport.h` | `importMeshHierarchyParts`, `importBrepHierarchyParts`, `importMeshFileExtended`, `HierarchyMeshImportResult` | Widget `MainWindowImportCaptureRenderController`；Facade 路由 | **Internal**→Facade 封装后对外仍用 Facade |

**DEVELOPER_GUIDE 锚点**：§4.2a，§4.4.1–§4.4.1b，§4.4.3 迁移表。

---

## D — Project IO

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/ProjectPackageIo.h` | `buildProjectSaveRoot`, `finalizeProjectLoadFollowAndViewport`, `applyProjectViewportFromJson`, `restoreRobotKinematicsFromProjectJson`, `applyRestoredJointAnglesToScene`, `load/mergeRobotPrograms*`, `mergeRobotKinematicsIntoProjectRoot` | Widget 工程打开/保存 | **Stable**（桌面工程编排） |
| `inc/BackendProjectObjectIo.h` | `saveProjectObject`, `loadProjectObjectsFromJson`, `registerEmbeddedProjectObject`, `parse/applyProjectEdges*`, `applyProjectEdgesFollowBindingAndSolve`, `exportBackendTriangleSoupMm`, … | Widget；Gateway（soup 导出）；Internal | **Stable**（objects/edges）；部分同步 OSG 细节 **Internal** |
| `inc/AnnotationProjectIo.h` | `buildAnnotationsJsonFromOsg`, `applyAnnotationsFromProjectJson` | Widget 工程 IO；`ProjectPackageIo` | **Internal**（经 Package 编排） |
| `inc/RobotProjectKinematicsRestore.h` | `collectRobotLinkMeshBackendIds`, `restorePerLinkRobotKinematicsFromProjectJson` | `ProjectPackageIo` / Widget 加载 | **Internal** |
| PluginHost `DocumentPointCloudOps.h`（导出子集） | `exportPointCloudToPly`, `exportBrepToStep` | Widget 树右键；Plugin/AI | **Stable**（导出公共 API） |

**DEVELOPER_GUIDE 锚点**：§4.2c，§4.4.2 工程 I/O 表。

---

## E — Follow / Events / Visual sync

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/DocumentHostEvents.h` | `publishBackendObjectRegistered/Removed`, `publishRobotKinematicsApplied`, `publishSelectionChanged`, `publishPoseCommitted*`, `publishProjectLoaded` | Internal adapters；Widget 订阅 `EventHub` | **Stable**（事件出口约定）；自由函数头偏编排 **Internal** |
| `inc/BackendFollowSolve.h` | `FollowSolveContext`, `runBackendFollowSolveAndSync`, `afterFollowPropertyEdited` | Widget 帧循环 / 属性；`IDataService::runFollowSolveAndSync`；Gateway（osg=null） | **Stable**（求解入口）；Context 守卫注入 **Internal** |
| `inc/BackendHierarchyFollow.h` | `applyHierarchyFollowBinding` | Widget / 工程 edges；**跳过** URDF kinematics-owned | **Stable**（绑定语义） |
| `inc/BackendVisualSync.h` | `propertyKeyNeedsVisualSync`, `propertyKeyCommitsPose`, `syncVisualAfterPropertyChange*`, `afterDataServicePropertyChange` | `DataServiceAdapter`；Widget 属性分量编辑 | **Internal**（契约后处理） |

**DEVELOPER_GUIDE 锚点**：§4.2a Follow，§4.2b VisualSync，§4.4.2 Events；位姿所有权硬边界见 §4.4.1。

---

## F — Robot

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/IRobotUrdfImportContext.h` | `IRobotUrdfImportContext`（Backend/场景加载/旁路表/per-link binding/坐标系…） | `UrdfRobotImport`；`DocumentPage` / `HeadlessRobotContext` 实现 | **Stable**（导入边界接口） |
| `inc/UrdfRobotImport.h` | `importUrdfRobot` | `RobotServiceAdapter`；Headless/桌面 | **Stable** |
| `inc/RobotPlanInstruction.h` | `planMotionInstruction`, `planRobotInstruction` | `RobotServiceAdapter`；Widget `RobotSimulationController` | **Stable** |
| `inc/RobotProgramJsonIo.h` | `robotProgramsToJson` / `robotProgramsFromJson` | Adapter / `ProjectPackageIo` | **Internal**（经 `robot()` JSON API） |
| `inc/RobotCoordinateFrameOps.h` | `captureTool/UserFrameFromTcpPose`, `resetActiveToolFrame`, `buildFrameOverlaySnapshot`, frame CRUD/JSON | Gateway/Headless；桌面可共路 | **Stable**（双端共路目标） |
| `inc/IRobotInstructionPropertyDelegate.h` | `IRobotInstructionPropertyDelegate` | Widget 注入；`RobotServiceAdapter` 转发 | **Stable**（委托边界） |
| `inc/RobotInstructionPropertyDto.h` | JSON → `PropertyRowDto` / feasible axis DTO | Internal / Headless delegate | **Internal** |
| `inc/IPerLinkKinematicsHost.h` | `applyPerLinkRobotFkFromGizmoAnchor`, `reconcilePerLinkOuterBindFromScene` | `DocumentPage` / Host impl | **Stable**（解耦接口） |
| `inc/IPerLinkRobotStateAccessor.h` | `PerLinkRobotStateSnapshot`/`FkResult`, `extract`/`apply`, pose sink | `PerLinkKinematicsHostImpl`；`DocumentPage` | **Stable**（DIP） |
| `inc/PerLinkKinematicsHostImpl.h` | `PerLinkKinematicsHostImpl` | DocumentPage 注入 | **Internal**（实现类） |

**DEVELOPER_GUIDE 锚点**：§4.4，§4.4.4 URDF，§4.4.5 per-link。

---

## G — Headless（Gateway / Web 共路）

| Header | Key symbols | Primary callers | Class |
|--------|-------------|-----------------|-------|
| `inc/HeadlessRobotContext.h` | `HeadlessRobotContext`, `BackendDataPoseSink`；FK/IK/TCP 捕获；`IRobotUrdfImportContext` + `IRobotSimulationDocument` | Gateway；`createHeadlessDocumentHost` | **Stable** |
| `inc/HeadlessTrajectorySession.h` | PathPlan 绑定、特征离散、pipeline preview/apply、模板 CRUD… | Gateway `/api` 轨迹 | **Stable** |
| `inc/HeadlessPointCloudBridge.h` | `infoJson`/`measureJson`/preview&chunk soup；downsample/crop/register/reconstruct/surface… | Gateway 点云 REST | **Stable** |
| `inc/HeadlessInstructionPropertyDelegate.h` | `HeadlessInstructionPropertyDelegate` | Headless `DocumentHost` 自持有 | **Stable** |

工厂：`createHeadlessDocumentHost`（§A）启用无 OSG / Null `IRenderView` 路径。

---

## H — Plugin / AI / Osg compiled-in

> 路径在 `src/UI/CloudSimPluginHost/` 与 `src/UI/Widget/`，**编入** `CloudSimHost.dll`（非独立 DLL）。插件消费方仍只链 **PluginSDK / AiSDK**。

### H.1 PluginHost（桌面）

| Header（PluginHost/inc） | Key symbols | Primary callers | Class |
|--------------------------|-------------|-----------------|-------|
| `PluginManager.h` | `PluginManager`（`loadAllFromPluginsDirectory`, `shutdownAll`, `invokeProjectAboutToSave/Loaded`） | Widget `MainWindow` 启动 | **Stable**（白名单） |
| `PluginHostContext.h` | `PluginHostContext` → `IPluginHostContext` | PluginManager；插件经 SDK 虚接口 | **Internal**（实现；插件不 include） |
| `IPluginMainWindowHost.h` | `IPluginMainWindowHost` | Widget 实现；PluginHostContext | **Internal**（Host↔Widget 解耦） |
| `PluginDocumentAdapter.h` / `PluginSceneBridgeAdapter.h` | 文档/场景桥到 `DocumentHost` | PluginHostContext | **Internal** |
| `PluginGeometryHostImpl.h` / `PluginPointCloudHostImpl.h` / `PluginLabelingHostImpl.h` / `PluginDelegatedBackend.h` | Domain Host 实现 | Plugin / AI | **Internal** |
| `DocumentGeometryOps.h` | `document_geometry_ops::*`（注册 soup、布尔参数转换…） | Geometry Host / AI Mesh | **Internal** |
| `DocumentPointCloudOps.h` | 点云/网格查询、配准、重建、**`exportPointCloudToPly`/`exportBrepToStep`** | PointCloud Host / Widget / AI | 导出两函数 **Stable**；其余 **Internal** |
| `plugin_host_global.h` | `PLUGIN_HOST_STATIC` | 构建 | **Internal** |

### H.2 AI（编入 Host）

| 区域 | Key symbols（代表） | Primary callers | Class |
|------|---------------------|-----------------|-------|
| `Ai/AiAssistantHostImpl.h` | `AiAssistantHostImpl` → `IAiAssistantHost` | PluginHostContext | **Internal**（SDK 面 Stable） |
| `Ai/AiActionPlanExecutor.h`, `AiHostButtonApiDispatch.h` | ActionPlan / 按钮 keyword 分发 | AI Dock | **Internal** |
| `Ai/AiAgentRuntime.h`, `AiAgentPlanBuilder.h`, `AiAgentMemory.h`, `AiAgentTrace.h`, `AiAgentPickDialog.h` | Agent 运行时 | AI ConfirmPanel | **Internal** |
| Domain handlers | `CatalogActionPlan*`, `MeshCreate*`, `MeshCompose*`, `FeatureCompose*`, `GeometryRecognize*`, `TrajectoryFeature*` | DomainRouter | **Internal** |
| LLM/config | `AiLlmClient`, `AiConfigLoader`, `AiIntent*`, `AiDomainRegistryImpl`, `AiSceneSnapshotBuilder`, … | AI 管线 | **Internal** |

插件/AI **对外 Stable** = PluginSDK / AiSDK 接口；Host 内实现类不进 Stable ABI 白名单。

### H.3 OsgWidget 壳（Widget 源、Host 编译）

| 代表单元 | 角色 | Callers | Class |
|----------|------|---------|-------|
| `OsgWidget` + Controllers / `QWidgetViewer` / `ObjectTransformOperation` | Qt↔OSG 壳；核心场景在 `OsgWidgetCore.dll` | `DocumentHost`；Adapter；存量 `widgetOsgFromPage` | **Internal**（实现）；契约出口为 `IRenderView` **Stable** |
| `OsgWidgetSceneBridge`, `BackendFollowReverseIndex`, `BackendSceneDocumentFacade` | 文档级场景/跟随索引 | `DocumentHost`；PluginSceneBridge | **Internal** |

**DEVELOPER_GUIDE 锚点**：§2 编入源码列表，§4.7 Bridge/Index，§7 → PluginHost / AiSDK / OsgWidgetCore 指南。

---

## Header inventory — `CloudSimHost/inc`（全量）

| Path | Domain |
|------|--------|
| `CloudSimHost.h`, `DocumentHost.h`, `DocumentProjectSidecar.h`, `DocumentFollowState.h`, `HostRenderViewFactory.h`, `DocumentHostAccess.h` | A |
| `cloudsim_host_global.h`, `widget_global.h`, `pch.h` | build |
| `adapters/DataServiceAdapter.h`, `OsgRenderViewAdapter.h`, `RobotServiceAdapter.h`, `SelectionVisualService.h` | B |
| `DocumentImportFacade.h`, `BackendFileImport.h`, `HierarchyMeshImport.h` | C |
| `ProjectPackageIo.h`, `BackendProjectObjectIo.h`, `AnnotationProjectIo.h`, `RobotProjectKinematicsRestore.h` | D |
| `DocumentHostEvents.h`, `BackendFollowSolve.h`, `BackendHierarchyFollow.h`, `BackendVisualSync.h` | E |
| `IRobotUrdfImportContext.h`, `UrdfRobotImport.h`, `RobotPlanInstruction.h`, `RobotProgramJsonIo.h`, `RobotCoordinateFrameOps.h`, `IRobotInstructionPropertyDelegate.h`, `RobotInstructionPropertyDto.h`, `IPerLinkKinematicsHost.h`, `IPerLinkRobotStateAccessor.h`, `PerLinkKinematicsHostImpl.h` | F |
| `HeadlessRobotContext.h`, `HeadlessTrajectorySession.h`, `HeadlessPointCloudBridge.h`, `HeadlessInstructionPropertyDelegate.h` | G |

Plugin/AI/Osg 见 §H（不在 `CloudSimHost/inc`，但同 DLL）。

---

## Caller matrix（摘要）

| 消费方 | 应走 | 应避免 |
|--------|------|--------|
| Widget 新代码 | `data()`/`robot()`/`render()`/`events()`；ImportFacade；ProjectPackageIo；Follow 经契约 | 扩散 `BackendDataManager*`、直接 OSG 头 |
| Gateway | `createHeadless*` + Headless* + Core | 依赖 `PluginManager` / OsgWidget |
| Plugin | PluginSDK only | `#include` Host/PluginHost 实现头 |
| AI | AiSDK + Host 内 Impl（同 DLL） | 插件 DLL 直链 Host 符号（除经 SDK） |
| Host Internal | FileImport / VisualSync / osgWidgetFrom | 对外再导出新自由函数而不经 Facade/Core |

---

## Notes / 演进挂钩

1. **同 DLL 逻辑域**（Wave2 已落地）：`inc|source` 按 `import/project/robot/headless/follow` 分目录，根目录 shim 保扁平 `#include`。  
2. **DocumentHost 瘦身**（Wave3 已落地）：旁路表 / Follow 运行时 → `DocumentProjectSidecar` / `DocumentFollowState`；公开 API 仍经 `DocumentHost` 转发。  
3. **`backend()`**：外部调用点见 `BACKEND_CALLSITE_INVENTORY`；白名单只减不增；已上提样板 `findByClassName` + `documentData()`。  
4. **物理拆 DLL**：Plugin/Osg **延期**；AiHost / Web 链接期去 OSG 仅按需（见 `OPTIONAL_EVAL_WebH2_AiHost.md`）。  
5. **空间契约**：URDF / 层级 / 配准写回 → `BackendWorldPose` / `spatial_contract_world_pose.md`。
``
