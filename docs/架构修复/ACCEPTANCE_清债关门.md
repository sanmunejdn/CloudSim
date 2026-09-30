# ACCEPTANCE — 全量清债关门（档 3）

Bug：#8

## 窗口对照

| 窗口 | 内容 | 结果 |
|------|------|------|
| D1 | Bootstrap Utility 仅头文件；删除 stub `ApplicationContext` | 完成；Debug+Release Bootstrap OK |
| D2 | Registry 业务路径零静默兜底（Importer / Backend / Codec / Trajectory / Feature）；Host 访问器 assert | 完成；ConfigRegistry / `BackendDataManager` 另轨余留 |
| D3 | Mate→`IRobotOsgViewHost`；DocumentPage→`IOsgWidgetView`；`IViewportSceneSignals`；`SelectionOperation` 迁 Host | 完成；Widget 生产路径无裸 `OsgWidget*`（仅适配层） |
| D4 | `items.props` 内联 Core 后删除；check 以 Core 为真相 | 完成；host-headless check OK |
| D5 | `LabelingSession`/`LabelingTypes` 迁 Host Core；Host 不链 LabelingSDK | 完成；LabelingSDK 保留 stub DLL |
| D6 | `BackendVisualRegistry` 实例 + `setProcessInstance` + 组合根注入 | 完成 |
| D7 | `sdkAbiVersion` + IID `0x00013900`；web fallback 护栏；SDK §1/§4 文档关门 | 完成；插件 Rebuild 后 IID 扫描 20 DLL OK |
| D8 | 本文 + BugTracker | 完成 |

## 余留（明示不阻塞关门）

- `BackendDataManager::instance`（无 process 槽；已有 `makeBackendManagerDataService(override)`）
- `TrajectoryOpConfigRegistry` / `FeatureDiscretizerConfigRegistry` 仍为配置侧单例
- `IPluginHostContext` 旧虚表转发层（设计冻结，不删）
- LabelingSDK stub DLL 仍在 sln（勿再链接会话逻辑）

## 编译验证（代表性）

- CloudSimBootstrap / HostCore / Host / Headless / Widget / Data / BackendVisual / RobotUrdf / OsgWidgetCore / PluginSDK / 全部插件：Debug\|x64 + Release\|x64（各窗口验收时已编）
- `python scripts/make-source-check.ps1`：host-headless、plugin-iid、web-fallback-freshness 等

## 回归建议

1. 桌面：开文档、导入 STEP/网格、装配拾取、视口工具条、示教/剖面、属性面板联动
2. 标注：Labeling 会话创建/导出（Host 内 Session）
3. 插件：加载全部插件（IID `0x00013900` + `sdkAbiVersion`）
4. Web：Vite `bin/*/web/assets/*.js`；勿设 `CLOUDSIM_WEB_FALLBACK`
