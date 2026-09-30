# ACCEPTANCE：终态 A+B+C

BugTracker **#9**。窗口验收（均为 Debug|x64 + Release|x64）。

| 窗口 | 内容 | 结果 |
|------|------|------|
| T1 | 嵌入 API 自持 `BackendDataManager`；`instance()` abort | 通过 |
| T2 | ConfigRegistry `setProcessInstance`；`DocumentHost::configResourceBaseDir`；builtins `register*(Registry&)` | 通过 |
| T3 | `osgWidget()` 私有；`SelectionOperation` → `IOsgWidgetView*` | 通过 |
| T4 | 四分面 MI；Core 去 CaptureController include；DocumentHost 分面访问器 | 通过 |
| T5 | `CloudSimHostDesktopViewport` 工程 + DESKTOP_ONLY 迁入 | 通过 |
| T6 | Host/Widget 链 Viewport；Headless 不链；check 三方对齐 | 通过 |
| T7 | WHOLEARCHIVE 仅 Core→Host/Headless；Viewport 无 WHOLEARCHIVE；dumpbin：`OsgWidget` 在 Viewport 导出 | 通过 |
| T8 | DESIGN/ACCEPTANCE + 本表 | 通过 |

## 成功标准核对

1. 无生产 `BackendDataManager::instance`；嵌入自持；Host 每文档 `m_backend`；Config schema 进程注入；`resourceBaseDir` 经 DocumentHost。
2. `DocumentHost` 无公开 `osgWidget()`；SelectionOperation 不持 `OsgWidget*`。
3. 分面接口存在；`IOsgWidgetView` 为其聚合。
4. 存在 `CloudSimHostDesktopViewport.dll`；桌面源不在 Host/Headless/Core 清单；`check_host_headless_sources.py` 绿。
5. WHOLEARCHIVE 仅 Core→Host/Headless。
6. 相关工程双配置绿。

## 回归建议

- 多文档切换后轨迹/离散化 Config JSON 仍从各文档 `configResourceBaseDir` 加载
- 桌面：开文档视口、装配拾取、视口工具栏
- Headless Web：启动与轨迹 schema API

## 余留

- ~~D：插件旧虚表 major 收缩 / LabelingSDK 物理退场~~ → 见 [ACCEPTANCE_终态余留全量.md](ACCEPTANCE_终态余留全量.md)（#10）
- ~~E：生命周期自动化 CI~~ → 同上（`test_lifecycle_contract.py`）
- ~~Host↔Viewport 循环依赖~~ → 同上（Viewport 不再链 Host.lib）
- Core `/WHOLEARCHIVE` 仍保留（再导出契约；本轮不做消灭）
