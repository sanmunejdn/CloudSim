# TODO — 仿真统一内核

| 项 | 说明 | 优先级 |
|----|------|--------|
| 填充挂载边 | 从 `CustomDeviceRobotMount` 同步到 `WorldApplyOrder::setMountEdges` | P1 |
| CollisionGuard probe | 在 Controller 注入 `BackendCollisionSync` 检测回调并开放 UI 策略 | P1 |
| Host 多 Owner sink | `HostIoSignalNetworkAdapter::sink(ownerId)` 返回每 Owner 表 | P2 |
| devicePrograms 工程 IO | 已由薄 CEE #68 落地 MainWindow 存取 | 完成 |
| Run All 规划并行 | 多机急算/lookahead 后台 Job 分桶，避免串行卡顿 | P2 |
| 设备指令 UI | DeviceCommandPage 插入 GoNamedPose/SyncBarrier 等 | P2 |

操作指引：挂载边可在统一 Run 启动前遍历文档 CustomDevice 的 Mount 组件；devicePrograms 侧车键名 `devicePrograms`，与 CONSENSUS 一致。
