# ALIGNMENT — UI 思想落地

## 原始需求

按 EUI 对照审查的 10 条原则，补齐 CloudSim 桌面 Qt + Web 的契约缺口：Job 可取消、关文档生命周期、指令树稳定 id、Web 增量刷新、身份/缓存/Token、长列表虚拟化。不换 Qt 主壳。

## 代码基线（2026-08）

| 原则 | 结论 | 已落地 | 仍缺 |
|------|------|--------|------|
| 状态分层 | 部分 | Widget/Host/Core 分层 | UI 可直接 backend() |
| 受控单向流 | 部分 | 属性/指令属性按 instructionId | 程序树 kInstrPtrRole |
| 高频不碰业务 | 桌面较好/Web 弱 | 防抖、Units patchVisible | sceneStore 全量 refresh |
| 生命周期 | 部分 | 关 Tab removeDocument | 无 onDocumentClosed |
| 派生现算 | 部分 | PlanResultCache | 可行轴成员缓存双份 |
| 异步 cancel | 弱 | 内部 jobId | 对外 void、无 cancel |
| 稳定身份 | 部分 | AI tab key、modeId | PluginDock_title、指针 hex |
| Token | 部分 | ApplicationStyle/UiIcons | 无公开 tokens、散落 hex |
| 虚拟化 | 弱 | 特征表 Model | Units/程序树全量 item |
| 契约文档 | 强 | 各模块 DEVELOPER_GUIDE | 缺 lifetime/cancel 条款 |

## 边界

| 做 | 不做 |
|----|------|
| P0–P2 按计划落地 | 不引入 EUI-NEO / 不换 Qt |
| ABI 仅 vtable 末尾追加（1.52 / 0x00013400） | 不破坏旧 enqueueJob |
| 复用 GET /api/objects/{id}、findInstructionById | 不重复实现已有 API |

## 疑问澄清

无未决决策；按计划默认方案执行。
