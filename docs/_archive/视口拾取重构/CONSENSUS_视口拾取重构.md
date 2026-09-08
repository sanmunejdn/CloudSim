# CONSENSUS — 视口拾取重构

## 验收标准

1. `OsgWidget::eventFilter` 拾取分发只问 `ViewportInteractionController`。
2. 无交叉 `m_*PickMode` 互斥丛林（模式态由 activeTool 表达；旧 setter 仅门面）。
3. 业务经 Session 或明确 Op；有 Session 时不双投全局 `meshPickCommitted` 消费者。
4. `setPickHandler` 转发 resolved backendId，与 SelectionService 同 Op 串行、不双投。
5. Debug|x64 + Release|x64 Widget 及相关工程通过；孤儿源文件为 0。

## 技术约束

- PickEngine 唯一调用 `OsgScene::queryPick` / 高亮 API。
- Policy：GizmoAxis → RobotObjectSelect → Passthrough。
- 选择语义 vs `resolvePickScopeBackendId` 几何 scope 分离。
- 新文件筛选器：`inc|src\ViewportInteraction\...`。
