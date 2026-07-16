# 离散网格密度控制 - ACCEPTANCE

## 恢复检查（2026-07-14，非 progressive）

| 项 | 状态 | 说明 |
|----|------|------|
| Types/SDK density 字段 | 完成 | `MeshDensityControl` / `PluginMeshDensityControl` |
| Host 透传 | 完成 | `DocumentGeometryOps::toGeoMeshParams` |
| 面数二分 | 完成 | GeometryAlgorithm ~8 iter / ±15% |
| 边长 OCC×0.25 + refine | 完成 | 非偏粗基网格 |
| Data 单次 remesh | 完成 | 失败保留 refine；>400k 跳过 remesh |
| 插件 UI | 完成 | 三选一 + avgEdge 状态 |
| SelfTest | 完成 | 面数 + 边长 refine |
| 无 progressive / 无粗 OCC / 无 remesh 失败再 refine | 完成 | 已确认代码路径 |

## 需本机 VS 验证

- [ ] Debug/Release 全量链接 Host
- [ ] 实机 STEP：边长/面数结果与形态可接受
