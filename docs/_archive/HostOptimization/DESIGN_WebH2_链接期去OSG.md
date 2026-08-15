# DESIGN — WebH2 链接期去 OSG（Phase1）

## 架构

- `CloudSimHostHeadless.dll`：与 `CloudSimHost` 同源，宏 `CLOUDSIM_HOST_HEADLESS_ONLY`
- `inc/headless_stub/OsgWidget.h` 覆盖 Widget 真头，满足编译、运行期无 OSG 视口
- `OsgWidgetSceneBridge_Headless.cpp` 替代 `OsgWidgetSceneBridge.cpp`
- Web 链：`CloudSimWeb.exe` → `CloudSimWebGateway` → `CloudSimHostHeadless`
- 桌面链不变：`CloudSim.exe` → `CloudSimHost` → `OsgWidgetCore`

## 关键裁剪

| 类别 | Headless 处理 |
|------|----------------|
| 链接 | 无 `OsgWidgetCore.lib` / ProjectReference |
| 编译单元 | 排除全部 `OsgWidget*.cpp`、交互 `*Operation.cpp`、`OsgRenderViewAdapter` |
| 工厂 | `createHostRenderViewFactory()` → `makeNullRenderViewFactory()` |
| DocumentHost | `#ifndef HEADLESS_ONLY` 才构造真 `OsgWidget` |

## Phase2（未做）

- 去掉 `RobotUrdf`→`BackendVisual` / Host→`osg*.dll` 传递依赖
