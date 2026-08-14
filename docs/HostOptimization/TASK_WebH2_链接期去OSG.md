# TASK — WebH2 Phase1

## T1 工程与桩代码
- [x] `CloudSimHostHeadless.vcxproj`（GUID `…90AC`，`TargetName` Headless）
- [x] `headless_stub/OsgWidget.h`
- [x] `OsgWidgetSceneBridge_Headless.cpp`

## T2 源码条件编译
- [x] `DocumentHost.cpp` / `HostRenderViewFactory.cpp` 宏守卫

## T3 Web 解决方案接线
- [x] `CloudSimWeb.sln` + Gateway / Web → Headless（**Bootstrap 不绑 Host/Headless**，仅 Core；避免桌面/Web 共享桩被绑死）

## T4 验证
- [x] Debug|x64 + Release|x64：`CloudSimHostHeadless`、`CloudSimWebGateway`、`CloudSimWeb`
- [x] 桌面 `CloudSimHost` 双配置仍过
- [x] `dumpbin /dependents CloudSimHostHeadless.dll` 无 `OsgWidgetCore.dll`

> 复验日期：2026-08-14（轨迹 AI 确认离散 / 候选高亮 / catalog 不截断 / PostBuild `CloudSimBinDir` 之后）。  
> Headless 仍可传递依赖 `osg161-osg.dll`（Phase2：RobotUrdf/BackendVisual），**不**算 Phase1 失败。  
> Web PostBuild 已用 `$(CloudSimBinDir)web`；本轮复验设 `CLOUDSIM_WEB_SKIP_BUILD=1` 仅验链接。

## 与近期 Host 改动的关系（不必为本任务再改代码）

| 近期能力 | 落点 | 对 WebH2 |
|----------|------|----------|
| AI 选特征→确认离散、对话框、`face_N` | `AiWidget` / 桌面 Dock | 不进 Headless 链接面；**无需**改 WebH2 |
| 边折线 / 面片 overlay、leader 锚点 | `OsgWidgetCore` + `OsgRenderViewAdapter`（桌面 Host） | Headless 排除 Adapter；`NullCoreServices` no-op；**无需**改 |
| catalog `maxItems=0` | `AiTrajectoryFeatureCatalog`（Headless 已编入） | 纯 JSON，无 OSG；**无需**改 |
| 网页若要对齐 AI 轨迹 | 新 Gateway API + React，预览走 JSON/Three.js | **须遵守** WebH2：禁止把 `OsgWidgetCore` 链回 Headless/Web |
