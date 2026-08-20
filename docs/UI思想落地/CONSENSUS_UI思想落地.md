# CONSENSUS — UI 思想落地

## 需求与验收

1. **Job cancel**：`JobSystem` 返回 id；`JobCancelToken`；SDK `enqueueCancellableJob`/`cancelJob`；旧 `enqueueJob` 保留。
2. **关文档**：`onDocumentClosed`；三插件 destroyOnClose 清 `m_pagesByDocId`。
3. **指令树**：`kInstrIdRole` 存 id，去掉裸指针身份。
4. **Web**：`PoseCommitted`/`ObjectPatched` 增量 merge；SSE 队列上限 256；`stop` 断 `visualSceneDirty`。
5. **P1**：稳定 dock objectName；可行轴并入 PlanResultCache；`ApplicationStyle::tokens`；桌面树减少全量 visibility。
6. **P2**：Units/程序树 Model 化；Web UnitsTree 阈值虚拟化。

## 技术约定

- ABI：`CLOUDSIM_PLUGIN_HOST_VERSION = 0x00013400`（1.52）
- 关文档默认 **destroyOnClose**；跨文档缓存不得挂 QWidget
- Job lambda 禁止捕获裸 QWidget*；用 id + QPointer + cancel
- C++ Core Guidelines + 纯中文 Why 注释
- 相关工程 Debug|x64 与 Release|x64 均须通过

## 任务边界

不重写属性面板声明式 compose；不做完整 token 代码生成器（P1 手工对齐 Web CSS 变量即可）。
