# CloudSimPluginSDK 开发指南

> **文档导航**：[全库入口](../../README.md) · [全量目录](../../README.md) · [开发手册](../../开发手册/01-总览.md) · [产品索引](../README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

## 定位

`CloudSimPluginSDK.dll` 是 **插件与宿主之间的唯一稳定 ABI**。插件工程只链接本 SDK，不得链接 `Widget.lib` / `RobotScene.lib`。

## 版本

- 宿主版本宏：`CLOUDSIM_PLUGIN_HOST_VERSION`（当前 `0x00013800` = 1.56.0；含 7 个窄上下文查询）
- IID：`CloudSimPlugin_iid` = `com.cloudsim.ICloudSimPlugin/1.0.0x00013A00`（写在 `ICloudSimPlugin.h`，**bump 后须 Rebuild 全部插件**，增量编译不会重跑 moc）
- SDK ABI：`CLOUDSIM_PLUGIN_SDK_VERSION` = `0x00013A00`（聚合虚表 major 砍刀：仅门面 + 窄 getter）
- `ICloudSimPlugin::sdkAbiVersion()`：vtable 末尾默认返回 `CLOUDSIM_PLUGIN_SDK_VERSION`；宿主在 IID 校验后比对 `cloudsimPluginSdkVersion()`
- `IPluginDocument`：`documentId()`、`removeBackendObject()`；**1.2.0+** `queryPointCloudInfo` / `measurePointCloud` / `exportMeshToPly`（UI 线程）
- `IPluginHostContext`（**0x00013A00+**）：聚合门面，**仅** `hostVersion` / `applicationDirPath` / `log*` / `useChinese` / `onLanguageChanged` + 7 个窄上下文 getter（`documentContext` / `uiContext` / `geometryContext` / `aiContext` / `robotContext` / `jobContext` / `projectContext`）。域 API（文档/UI/几何/AI/标注/点云/机器人/任务/工程钩子）只存在于对应 `IPlugin*Context` / `IPlugin*Host`
- 清单 `plugin.json` 中 `minHostVersion` 使用字符串 `"1.0.0"`
- 运行时调用 `IPluginHostContext::hostVersion()` 比对

## 窄接口（0x00013A00+ 强制）

插件按域取窄上下文；聚合接口不再提供域虚函数：

```cpp
IPluginDocument* doc = host->documentContext()->activeDocument();
IPluginGeometryHost* geo = host->geometryContext()->geometryHost();
```

- HelloAiPlugin / GeometryPlugin：示范；其余插件亦须按域取窄上下文
- 评审约定：禁止对聚合接口调用已删除的域方法；新能力只加到对应窄接口末尾

| 规则 | 说明 |
|------|------|
| 只用窄接口 | `documentContext()` / `uiContext()` / `geometryContext()` / `aiContext()` / `robotContext()` / `jobContext()` / `projectContext()` |
| 聚合是门面 | 仅基础槽 + getter；域能力不在聚合虚表 |
| 版本防御 | 调用窄查询前先比 `hostVersion()`；不足时勿调用（vtable 越界） |
| ABI 红线 | 窄接口虚函数只许**末尾追加**；禁止中间插入或同槽改签名 |

### ABI bump checklist

1. 同步改 `CLOUDSIM_PLUGIN_SDK_VERSION` / `_STR`（`cloudsim_plugin_sdk_global.h`）与 `CloudSimPlugin_iid` 字面量（`ICloudSimPlugin.h`，moc 只吃字面量）
2. 若动 Ai 域：同步 `CLOUDSIM_AI_SDK_VERSION` / `_STR` 与 `CloudSimAiPlugin_iid`
3. **Rebuild**（或清 `bin/*/middle/*/qt/moc` 后编）全部相关插件；增量编常不重跑 moc
4. 跑 `python scripts/check_plugin_iid_alignment.py`（已串入 `make-source-check.ps1`）

## 工具链约束

| 项 | 要求 |
|----|------|
| 平台 | x64 |
| 工具集 | v142（VS 2019） |
| Qt | 5.14.2_msvc2017_64（与 CloudSim 一致） |
| 运行时 | 将 `CloudSimPluginSDK.dll` 与插件 DLL 放在 exe 同目录或 `plugins/<id>/` |

## 插件契约

实现 `ICloudSimPlugin` 并导出 Qt 插件：

```cpp
class MyPlugin : public QObject, public ICloudSimPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID CloudSimPlugin_iid)
    Q_INTERFACES(ICloudSimPlugin)
    // pluginId(), displayName(), initialize(), shutdown()
};
Q_IMPORT_PLUGIN(MyPlugin) // 仅静态测试时需要
```

## 清单 `plugin.json`

与 exe 同级目录：`plugins/<id>/plugin.json`

```json
{
  "id": "com.example.myplugin",
  "name": "MyPlugin",
  "version": "1.0.0",
  "minHostVersion": "1.0.0",
  "library": "MyPlugin.dll",
  "enabled": true
}
```

## 宿主 API 摘要

| API | 说明 |
|-----|------|
| `registerSidePanelTab` | 右侧面板新页签（与 Workspace/AI 并列，**推荐**，避免与仿真 Dock 重叠） |
| `unregisterSidePanelTab` | 移除页签（`shutdown` 时调用） |
| `registerDockWidget` | 浮动 Dock（勿用 `Right`，会与工作区重叠） |
| `registerMenuPath` / `registerAction` | 菜单与动作 |
| `importFileIntoActiveDocument` | 活动文档导入文件；返回 root `backendId`（UTF-8） |
| `createPrimitiveMesh` | box/cylinder/cone/sphere → Host 注册 + OSG |
| `registerBackendType` | 自定义 `className`（`PluginDelegatedBackend`）；**1.54.0+** 可填 `propertyBindings`（Binding 真源），空则回退 `propertyRowsJson` |
| `registerTriangleMesh` | 三角 soup → `registerAdoptedMesh` |
| `enqueueJob` / `invokeOnUiThread` | 线程边界 |
| `pointCloudHost()` | **1.2.0+** 点云算法宿主（见下节） |
| `geometryHost()` | **1.5.0+** 几何算法宿主（STEP/BRep 离散、求交、布尔） |
| `geometryHost()->listComputableBackends` | **1.7.0+** 枚举活动文档可计算 STEP/BRep 后端 |
| `geometryHost()->pickStepElementFromViewport` | **1.7.0+** 3D 视图一次拾取 edge/face 并返回 `PluginGeometryStepRef` |
| `useChinese()` | 与主窗口 **Settings → Language** 一致（默认中文） |
| `onLanguageChanged(callback)` | 语言切换时 UI 线程通知插件 |
| `setSidePanelTabTitle(widget, titleUtf8)` | 更新侧栏 Tab 标题 |
| `IPluginDocument::queryPointCloudInfo` / `measurePointCloud` | **1.2.0+** 点数、包围盒、度量（UI 线程） |
| `IPluginDocument::exportMeshToPly` | **1.2.0+** 导出 `Model` 三角网格为 PLY（含 face） |

## 点云 SDK（1.2.0+）

头文件：[`PluginPointCloudTypes.h`](inc/PluginPointCloudTypes.h)、[`IPluginPointCloudHost.h`](inc/IPluginPointCloudHost.h)。

| `IPluginPointCloudHost` 方法 | 说明 |
|------------------------------|------|
| `downsamplePointCloudVoxel/Random` | 体素/随机下采样，原地写回 |
| `cropPointCloudByBox/Sphere` | AABB/球裁剪 |
| `pickPolylineFromViewport` | **1.11.0+** 3D 视图绘制封闭多边形（左键顶点、右键/双击闭合） |
| `cropPointCloudByPolyline` | **1.11.0+** 屏幕多边形裁剪（`keepInside` 保留/删除内部） |
| `applyRigidTransformToPointCloud` | 刚体变换（列主序 `PluginMat4`） |
| `removePointCloudOutliers` / `smoothPointCloudBilateral` | 离群/平滑 |
| `estimatePointCloudNormalsPca/Jet` / `orientPointCloudNormalsMst` | 法线估计与定向 |
| `preprocessPointCloudForReconstruction` | 重建前预处理 |
| `rigidRegisterPointCloudsIcp` | ICP 配准，可选应用到源 |
| `deformPointCloudTpsFromControls` / `deformPointCloudTpsFitAndDeform` | TPS 形变 |
| `reconstructMeshPoisson/PoissonAuto/ScaleSpace` | 重建 mesh 并 `registerAdoptedMesh` |
| `registerScanToCadTemplate` | **v2** 世界系反向 ICP；只更新模板 `worldMatrix`；cache 存 `icpRmseMm` + `templateWorldMatrixAtRegister` |
| `updateTemplateBrepFromAlignedScan` | **1.8.0+** 基于缓存逐面重构 → 新 `BrepModel`；`selectedFaceIndices` 空=全部面（见 `docs/ARCHIVE_ZIP_LOCATION.txt`） |
| `queryMeshInfo` | **1.9.0+** 查询网格面数/顶点数（UI 线程） |
| `simplifyMesh` | **1.9.0+** quadric-edge-collapse 简化，创建新 mesh |
| `smoothMesh` | **1.9.0+** Laplacian / Implicit Fairing 平滑 |
| `repairMesh` | **1.9.0+** 去退化面/重复顶点/非流形/填孔 |
| `remeshMeshIsotropic` | **1.9.0+** 各向同性重网格 |
| `analyzeMeshDefects` | **1.10.0+** 多信号缺陷检测（针状/突起/边界尖刺），回调 `PluginMeshDefectReport` |
| `clearMeshDefectHighlight` | **1.10.0+** 清除 `OsgScene::showMeshFaceHighlight` overlay |
| `reconstructSurfaceFromMesh` | **1.12.0+** 网格 → 新 `BrepModel`（全流程） |
| `beginMeshSurfaceReconstructSession` | **1.13.0+** 绑定 `docId + meshBackendId`，返回 `PluginMeshSurfaceReconstructSessionId` |
| `runMeshSurfaceReconstructStage` | **1.13.0+** 按 `PluginMeshSurfaceReconstructStage` 单步执行；回调 `PluginMeshSurfaceReconstructReport`（含 `stageSummaryZh`） |
| `clearMeshSurfaceReconstructSession` | **1.13.0+** 清除会话及临时 `*_预处理后` 网格 |
| `beginTubularGrindingSession` | **1.15.0+** 绑定 `docId + meshBackendId`，返回 `PluginTubularGrindingSessionId` |
| `runTubularGrindingStage` | **1.15.0+** 按 `PluginTubularGrindingStage` 单步执行；回调 `PluginTubularGrindingReport`（含 `stageSummaryZh` 与各阶段场景 backendId） |
| `clearTubularGrindingSession` | **1.15.0+** 清除会话及临时着色/辅助对象 |
| `nonRigidRegisterSpare` | **1.16.0+** SPARE 非刚性配准；源/目标可为点云或网格（`PluginPointCloudSpareParams`） |
| `nonRigidRegisterSdf` | **1.17.0+** SDF/DDF 混合非刚性配准（`PluginPointCloudSdfParams`） |
| `nonRigidRegisterPyramid` | **1.53.0+** 几何多分辨率金字塔（`PluginPointCloudPyramidParams`；网格↔网格） |

## 机器人运动宿主（1.55.0+）

头文件：[`PluginRobotTypes.h`](inc/PluginRobotTypes.h)、[`IPluginRobotHost.h`](inc/IPluginRobotHost.h)。

经 `IPluginHostContext::robotHost()` 获取（vtable 末尾追加）。用于读当前 TCP、选中物体基座位姿，以及多 TCP 路点 `planAndConfirmTcpWaypoints`（内部复用碰撞页 `planToTcpPose`）。插件禁止直链 `RobotScene` / `RobotPathPlanning`。

**ABI 注意**：`IPluginPointCloudHost` 新增虚函数**只能追加在接口末尾**。在中间插入会导致插件与宿主 vtable 错位（例如 `queryMeshInfo` 误调到 `simplifyMesh` 并崩溃）。升级 SPARE / SDF 接口后须**同时**重编 `CloudSimHost.dll` 与 `PointCloudPlugin.dll`。

## 分割标注 SDK（1.16.0+）

头文件：[`PluginLabelingTypes.h`](inc/PluginLabelingTypes.h)、[`IPluginLabelingHost.h`](inc/IPluginLabelingHost.h)。

经 `IPluginHostContext::labelingHost()` 获取。支持点云/网格交互标注（点选、刷选、套索）、Undo、导出 `pointnet-training` 数据集。UI 见 [`LabelingPlugin`](../LabelingPlugin/)。

回调均在 **UI 线程**。

## 线程

- `initialize` / `shutdown` / UI 回调：**UI 线程**
- 重 CPU 点云处理：调用 `pointCloudHost()->…`（宿主内部 `enqueueJob`）；**不要**在插件内直接调 pclalgo
- 界面语言：初始化时读 `useChinese()`；注册 `onLanguageChanged` 并在回调中刷新文案（与主窗口 **设置 → 语言** 同步）

## 示例

- [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../PointCloudPlugin/DEVELOPER_GUIDE.md)（点云导入、下采样、重建、模板 B-rep 更新）
- [`LabelingPlugin`](../LabelingPlugin/)（交互分割标注与训练 UI；会话引擎为 HostCore `LabelingSession`，**CloudSimLabelingSDK 已移除**）
- 插件开发模板：参见本指南中的 `ICloudSimPlugin` 接口定义和 `plugin.json` 示例
