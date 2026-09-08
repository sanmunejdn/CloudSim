# ACCEPTANCE：后端属性 Binding

## 实现对照

| 项 | 状态 |
|----|------|
| Domain typed API 当真源；Binding 为面板/schema 清单 | 通过 |
| 基类统一 `snapshotPropertyRows` / `applyPropertyChange`；删 `m_attributes` | 通过 |
| 全员 `visible`；Mesh `mesh.triangle_count`；Frame/CustomDevice `axisLengthMm` | 通过 |
| schema 由 Binding 生成；未知 key aspect=0；面板按 className 查 descriptor | 通过 |
| 停止 pose/color `syncPropertyBagFromState` | 通过 |
| `runBackendPropertyBindingSelfTest`（六类 builtin） | 通过（Debug 首次 schema once） |
| `DEVELOPER_GUIDE.md` §5 已更新 | 通过 |
| Data → CloudSimHost → Widget Debug\|x64 + Release\|x64 | 通过（2026-09-07） |

## 手工验收（建议）

- [ ] 点云/网格/Brep 面板：pose/rotation/color + `visible`，与树勾选同源
- [ ] Frame/CustomDevice：有轴长、无 color；改轴长后 OSG 轴更新
- [ ] 网页 GET `properties` 含上述行；PATCH `axisLengthMm` 可用
- [ ] 改 `pose.x` 仅脏 Transform；未知 key 不触发全量 OSG 同步

## 明确未做（按方案）

- RobotInstruction Attribute 管道
- Data ↔ Qt `Q_PROPERTY`
- 插件运行时 Binding 注册表
- 点云 `point_count` 等未要求行
