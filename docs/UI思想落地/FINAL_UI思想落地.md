# FINAL — UI 思想落地

## 总结

按 EUI 对照审查在 **不换 Qt** 前提下补齐契约：Job 协作取消、关文档销毁插件页、指令树稳定 id、Web SSE 增量与有界队列、Dock/侧栏身份、可行轴与 Plan 缓存合一、主题 token、Units 桌面 `QTreeView`+Model 与 Web 长列表限量渲染。

## ABI

`CLOUDSIM_PLUGIN_HOST_VERSION = 0x00013400`（1.52）：vtable 末尾追加 `enqueueCancellableJob`/`cancelJob`/`documentById`/`onDocumentClosed`；旧 `enqueueJob` 保留。

## 文档

- [ALIGNMENT](ALIGNMENT_UI思想落地.md)
- [CONSENSUS](CONSENSUS_UI思想落地.md)
- [ACCEPTANCE](ACCEPTANCE_UI思想落地.md)
- [TODO](TODO_UI思想落地.md)
