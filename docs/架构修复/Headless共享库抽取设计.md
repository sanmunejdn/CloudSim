# Headless 共享库抽取设计（CloudSimHostCore 静态库）

> 立项来源：[`P2_方向_中级缺陷.md`](P2_方向_中级缺陷.md) 架构缺陷分级修复，工作流 D 任务 3。
> 本文为**纯设计文档**，不改任何代码。结论先行：**当前不宜直接抽静态库**——共享源存在三处编译期分歧，须先做前置重构；短期推荐方案 A（零风险增强现状机制）。

## 1. 现状分析

### 1.1 工程结构

`CloudSimHost.vcxproj`（桌面 DLL）与 `CloudSimHostHeadless.vcxproj`（网页 DLL）位于同一目录 `src/Host/CloudSimHost/`，两者均已：

```text
<Import Project="CloudSimHostShared.items.props" />
```

即**源级共享已经存在**：`CloudSimHostShared.items.props` 是唯一共享清单（98 个 `ClCompile` + 95 个 `ClInclude`），两个工程各自编译同一份源。

`check_host_headless_sources.py` 的 `collect_items()` 会跟随相对 `.items.props` Import，共享清单天然对齐；脚本实际守护的是"直接写在某个 vcxproj 里的项"不漂移。

### 1.2 各工程专有项

| 类别 | 内容 |
|------|------|
| 仅桌面（vcxproj 直列） | `source\osg\*` 约 26 个（OSG 视口/拾取/控制器）、`adapters\OsgRenderViewAdapter.cpp`、`PluginPropertyBindingSelfTest.*` |
| 仅 Headless（vcxproj 直列） | `source\headless\OsgWidgetSceneBridge_Headless.cpp`、`inc\headless_stub\OsgWidget.h`（moc） |
| 两边直列的共享源（反例） | `source\osg\ViewportGestureRecognizer.cpp`、`source\pch.cpp` —— 未走 items.props，是漂移隐患 |

### 1.3 编译配置分歧（核心障碍）

| 分歧点 | 桌面 CloudSimHost | Headless CloudSimHostHeadless |
|--------|-------------------|-------------------------------|
| 预处理宏 | `CLOUDSIM_OSG_IN_HOST` | `CLOUDSIM_HOST_HEADLESS_ONLY` |
| include 顺序 | `inc` 在前 | **`inc\headless_stub` 在最前** |
| `OsgWidget.h` 解析 | `inc/osg/OsgWidget.h`（真 OSG 视口） | `inc/headless_stub/OsgWidget.h`（桩） |
| PCH | 各自 `pch.cpp` Create/Use | 同左 |

两个 stub/真头文件**故意共用同一头卫** `CLOUDSIMHOST_OSGWIDGET_H`（stub 注释：「与桌面 OsgWidget 同头卫，避免双定义」），靠 include 顺序二选一。

直接踩分歧的共享源（历史清单；宏分支已迁出）：

- ~~`#ifdef CLOUDSIM_HOST_HEADLESS_ONLY`：`DocumentHost.cpp` / `HostRenderViewFactory.cpp`~~ → 已改为 `HostOsgFlavorHooks_{Desktop,Headless}.cpp` 运行时/工厂注入（共享源无 flavor 宏）
- `#include "OsgWidget.h"` 的**共享源**仍依赖 include 顺序二选一；其中 `PluginGeometry/PointCloud/LabelingHostImpl` 因 Qt 信号需具体类型，**标为桌面专有例外**，不强迫全部进 `IOsgWidgetView`
- `IOsgWidgetView`：**冻结**「共享源新增能力只许接口末尾追加」；本阶段不拆分子接口、不建 `CloudSimHostCore`

## 2. 为什么"直接抽静态库"不可行

静态库只编译**一次**，只有一份宏定义和一份 include 顺序：

1. **宏分歧**：`CLOUDSIM_OSG_IN_HOST` 与 `CLOUDSIM_HOST_HEADLESS_ONLY` 互斥，静态库无法同时满足。
2. **同名头歧义**：20 个共享源经 include 顺序解析到不同的 `OsgWidget.h`，静态库只能解析到一个；且两头文件共用头卫，桌面 DLL 再链入静态库后，真 `OsgWidget.h` 会被已编译进 lib 的 stub 版语义污染（或反之）。
3. **导出宏无静态分支**：`cloudsim_host_global.h` 只有 `CLOUDSIM_HOST_LIB` 二分（export/import），没有 `BUILD_STATIC` 分支（`cloudsim_core_global.h` 有）。共享源中 `CLOUDSIM_HOST_EXPORT` 类移入静态库后，下游（插件 SDK 等）从 `CloudSimHost.dll` 导入符号的契约必须保持，导出宏须重新定义边界。

## 3. 方案对比

| 方案 | 内容 | 风险 | 结论 |
|------|------|------|------|
| **A 现状增强** | items.props 保持唯一事实源；check 脚本增加「共享源禁止直列 vcxproj」规则；把 `ViewportGestureRecognizer.cpp` 等反例收编进 props | 零 | **推荐，立即可做** |
| **B 目标态静态库** | 前置重构消除编译期分歧后，抽 `CloudSimHostCore.lib`，两 DLL 链接 | 高，需排期 | 长期目标，见 §4 |
| C 双静态库 | 同一份源以两种宏各编一个 lib | 编译次数不变、复杂度上升、无实质收益 | 不推荐 |

## 4. 方案 B 设计（目标态）

### 4.1 前置重构（按序，每步独立可编译验证）

1. **OsgWidget 依赖接口化**：共享源不直接 `#include "OsgWidget.h"`，改为依赖窄接口（如 `inc/IOsgWidgetView.h`，只含共享源实际用到的方法子集）。桌面由真 `OsgWidget` 实现，Headless 由 stub 实现，经构造注入。涉及 §1.3 列出的 20 个共享源——这是方案 B 工作量的主体。
2. **消除共享源宏分支**：**已完成**（`HostOsgFlavorHooks` + `DocumentHost`/`HostRenderViewFactory` 注入）。vcxproj 仍可保留 `CLOUDSIM_HOST_HEADLESS_ONLY` 供 moc/历史路径，但共享 `.cpp` 不得再依赖该宏。
3. **导出宏**：`cloudsim_host_global.h` 保留 `CLOUDSIM_HOST_STATIC`（纯静态消费方为空导出）与 `CLOUDSIM_HOST_LIB`（dllexport）。**落地取舍**：`CloudSimHostCore` 以 `CLOUDSIM_HOST_LIB` 编译，使对象文件带 dllexport；两 DLL 以 `/WHOLEARCHIVE:CloudSimHostCore.lib` 链入并再导出给 Widget/插件。若改用 `CLOUDSIM_HOST_STATIC` 则 Core 符号无导出属性，非 WHOLEARCHIVE 路径下易出现 Widget LNK2019。

### 4.2 CloudSimHostCore.vcxproj 结构

```text
src/Host/CloudSimHostCore/CloudSimHostCore.vcxproj
├─ ConfigurationType: StaticLibrary
├─ PreprocessorDefinitions: CLOUDSIM_HOST_LIB;WIDGET_LIB;AIBACKEND_LIB;...（无 flavor 宏；见 §4.1-3）
├─ 不使用 PCH（或独立 PCH；两个 DLL 的 pch 保持不变）
├─ ClCompile: 现 CloudSimHostShared.items.props 的 98 项（前置重构完成后）
└─ ClInclude: 现 95 项
```

两个 DLL 工程：`AdditionalDependencies` 增加 `CloudSimHostCore.lib`，`Link/AdditionalOptions` 增加 `/WHOLEARCHIVE:CloudSimHostCore.lib`；删除 items.props Import；`check_host_headless_sources.py` 校验 Core 清单与两 DLL 风味源对齐。

### 4.3 迁移步骤

1. 完成 §4.1 三步前置重构（每步 Debug|x64 + Release|x64 双编两个 DLL 验证）
2. 新建 CloudSimHostCore.vcxproj，迁入 items.props 全部条目，加入两个 sln
3. 两 DLL 链接 CloudSimHostCore.lib，删除 props Import，双编 + 跑通桌面/网页冒烟
4. 改造 check 脚本（§4.2），删除 `CloudSimHostShared.items.props`

## 5. 风险与缓解

| 风险 | 缓解 |
|------|------|
| OsgWidget 接口化改动面大（20 文件），引入行为回归 | 窄接口只暴露现用方法；逐文件迁移，每文件编译验证；不动 `OsgWidget.cpp` 本体 |
| 静态库符号可见性变化导致插件/Widget LNK2019 | Core 用 `CLOUDSIM_HOST_LIB` + DLL `/WHOLEARCHIVE`；核对 `CloudSimHost.dll` 导出表 |
| 宏改运行时注入后 flavor 行为漂移 | 桌面/网页两侧冒烟用例先行；`createHostRenderViewFactory` 模式已验证可行 |
| moc 分歧（两工程 QtMoc Defines 不同） | moc 产物留在各 DLL 工程，不进静态库 |

## 6. 结论

- §4.1-2 共享源 flavor 宏分支：已落地（`HostOsgFlavorHooks`）。
- §4.1-1 OsgWidget 接口化：共享 `.cpp` 已脱离 `OsgWidget.h`（`IOsgWidgetView` + observe*）；Plugin*HostImpl 拾取经窄观察 API。
- §4.1-3 / §4.2：已抽出 `CloudSimHostCore.lib`（`CloudSimHostCore.vcxproj`）；两 DLL 以 `CLOUDSIM_HOST_LIB` + `/WHOLEARCHIVE` 再导出；Host/Headless/Widget Debug|x64 与 Release|x64 已通过。
- `BackendVisualRegistry` 实例化与 UI `WidgetSceneSignalWiring` 全量去 `OsgWidget` 信号仍可后续迭代。
- 短期方案 A 检查脚本仍守护 Core / 两 DLL 风味源对齐。
