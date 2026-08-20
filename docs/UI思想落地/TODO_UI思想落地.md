# TODO — UI 思想落地

1. **指令程序树 Model**：在 id 身份稳定后改为 `QAbstractItemModel`，支撑超长程序滚动（拖放/IF-WHILE 仍绑在 `QTreeWidgetItem`）。
2. **CustomDeviceAssemblyDialog** 硬编码 QSS 全面改走 `ApplicationStyle::tokens`。
3. **插件迁移**：高频 `enqueueJob` 调用方可改用 `enqueueCancellableJob` + `cancelJob`（可选）。
4. **Web**：`fetchObjectDetail` 响应若补 `worldMatrix`，merge 后视口无需再全量拉列表。
5. **旧 settings.ini**：清理历史 `CloudSim.SidePanel.<hex>` 键（一次性迁移脚本可选）。
