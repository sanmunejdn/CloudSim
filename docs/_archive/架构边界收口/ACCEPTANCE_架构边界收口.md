# ACCEPTANCE：架构边界收口（Sprint A）

> 对应评审：[ARCHITECTURE_SUMMARY.md](../../ARCHITECTURE_SUMMARY.md) §11；画布评审结论 Sprint A。

## 范围

| ID | 项 | 验收标准 | 状态 |
|----|----|----------|------|
| P5 | PluginHost 文档一致 | §2 图、§2.1、§10、DIRECTORY_LAYOUT 均写「编入 Host」 | **通过** |
| P1 | OsgWidget 双轨清理 | `Host/inc/osg`、`Host/source/osg` 已删除；vcxproj 仍指向 `Widget/source` | **通过** |
| P4 | Follow 索引契约化 | `IDataService::followTargetId`（vtable 末尾追加）；`BackendFollowReverseIndex` 不再依赖 `BackendDataManager` | **通过** |
| P4b | 新代码禁区文档 | ARCHITECTURE §11.1 + Widget/Host DEVELOPER_GUIDE 写明禁区 | **通过** |

## 未纳入本 Sprint（显式推迟）

- `DocumentPage::backend()` / `IRobot*Host` 仍返回 `BackendDataManager&`（需先解 Host→RobotWidget）
- Widget 去 OSG include（阶段 3.3–3.4）
- `RobotSimulationController` 下沉 Host

## 验证清单

- [x] 生成 `CloudSimCore` → `CloudSimHost`（x64 Debug，via `CloudSim.sln`）
- [x] 生成 `Widget`（x64 Debug）
- [ ] 打开工程：跟随对象树/拖动目标后跟随者仍联动（需人工）
- [x] 确认仓库无 `src/Host/CloudSimHost/**/osg/**` 路径

## 变更摘要

- 删除 Host 下未编入工程的 OsgWidget 平行副本（约 36 文件）
- Core：`followTargetId` 追加到 `IDataService`
- Host：`DataServiceAdapter` / `NullDataService` 实现
- Widget：`BackendFollowReverseIndex` + `BackendSceneDocumentFacade` 经 `IDataService` 查跟随
