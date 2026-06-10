# CloudSimPluginSDK 开发指南

## 定位

`CloudSimPluginSDK.dll` 是 **插件与宿主之间的唯一稳定 ABI**。插件工程只链接本 SDK，不得链接 `Widget.lib` / `RobotScene.lib`。

## 版本

- 宿主版本宏：`CLOUDSIM_PLUGIN_HOST_VERSION`（当前 `0x00010B00` = 1.11.0）
- `IPluginDocument`：`documentId()`、`removeBackendObject()`；**1.2.0+** `queryPointCloudInfo` / `measurePointCloud` / `exportMeshToPly`（UI 线程）
- `IPluginHostContext`：`importFileIntoActiveDocument()`；**1.2.0+** `pointCloudHost()`；**1.4.0+** 末尾追加 `buildPrimitiveMeshSoup` / `booleanMeshSoups` / `booleanPrimitiveMeshes`；**1.5.0+** `geometryHost()`；**1.6.0+** `captureActiveViewportPng()`（活动文档 3D 视口 PNG，供 geometry.recognize 等多模态域）；**1.7.0+** `IPluginGeometryHost` 新增 `listComputableBackends` / `pickStepElementFromViewport`（几何插件可直接驱动后端对象 + 视图拾取）；**1.8.0+** `IPluginPointCloudHost` 将模板 B-rep 更新拆为 `registerScanToCadTemplate` + `updateTemplateBrepFromAlignedScan`（移除 `updateBrepFromCadTemplate`）；**1.9.0+** `IPluginPointCloudHost` 新增网格后处理：`queryMeshInfo` / `simplifyMesh` / `smoothMesh` / `repairMesh` / `remeshMeshIsotropic`（需宿主链接 `VcgAlgorithms.dll`）；**1.10.0+** `analyzeMeshDefects` / `clearMeshDefectHighlight`（只读缺陷分析 + 视口 overlay，不修改原网格）；**1.11.0+** `pickPolylineFromViewport` / `cropPointCloudByPolyline`（3D 视口多边形线框裁剪，屏幕投影）；新 API 均追加在 vtable 末尾，勿插入中间；升级宿主后须**重编译全部插件 DLL**
- 清单 `plugin.json` 中 `minHostVersion` 使用字符串 `"1.0.0"`
- 运行时调用 `IPluginHostContext::hostVersion()` 比对

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
    Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
    Q_INTERFACES(ICloudSimPlugin)
    // pluginId(), displayName(), initialize(), shutdown()
};
Q_IMPORT_PLUGIN(MyPlugin) // 仅静态测试时需要
```

## 清单 `plugin.json`

与 exe 同级目录：`plugins/<id>/plugin.json`

```json
{
  "id": "com.example.hello",
  "name": "Hello",
  "version": "1.0.0",
  "minHostVersion": "1.0.0",
  "library": "HelloPlugin.dll",
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
| `registerBackendType` | 自定义 `className`（`PluginDelegatedBackend`） |
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
| `registerScanToCadTemplate` | **1.8.0+** 扫描与 CAD 模板 ICP 配准；写回点云显示；Host 缓存对齐点云供下一步 |
| `updateTemplateBrepFromAlignedScan` | **1.8.0+** 基于缓存逐面重构 → 新 `BrepModel`；`selectedFaceIndices` 空=全部面（见 [`docs/template_brep_pointcloud_update.md`](../../docs/template_brep_pointcloud_update.md)） |
| `queryMeshInfo` | **1.9.0+** 查询网格面数/顶点数（UI 线程） |
| `simplifyMesh` | **1.9.0+** quadric-edge-collapse 简化，创建新 mesh |
| `smoothMesh` | **1.9.0+** Laplacian / Implicit Fairing 平滑 |
| `repairMesh` | **1.9.0+** 去退化面/重复顶点/非流形/填孔 |
| `remeshMeshIsotropic` | **1.9.0+** 各向同性重网格 |
| `analyzeMeshDefects` | **1.10.0+** 多信号缺陷检测（针状/突起/边界尖刺），回调 `PluginMeshDefectReport` |
| `clearMeshDefectHighlight` | **1.10.0+** 清除 `OsgScene::showMeshFaceHighlight` overlay |

回调 `PluginPointCloudFinishedFn` / `PluginMeshDefectFinishedFn` 在 **UI 线程**；算法在宿主 `enqueueJob` 内执行。插件 **不得** 链接 `PointCloudAlgorithm` / `Data`。

宿主实现细节：[`CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)。

## 线程

- `initialize` / `shutdown` / UI 回调：**UI 线程**
- 重 CPU 点云处理：调用 `pointCloudHost()->…`（宿主内部 `enqueueJob`）；**不要**在插件内直接调 pclalgo
- 界面语言：初始化时读 `useChinese()`；注册 `onLanguageChanged` 并在回调中刷新文案（与主窗口 **设置 → 语言** 同步）

## 示例

- [`HelloPlugin/DEVELOPER_GUIDE.md`](../HelloPlugin/DEVELOPER_GUIDE.md)（网格、`createPrimitiveMesh`）
- [`PointCloudPlugin/DEVELOPER_GUIDE.md`](../PointCloudPlugin/DEVELOPER_GUIDE.md)（点云导入、下采样、重建、模板 B-rep 更新）
