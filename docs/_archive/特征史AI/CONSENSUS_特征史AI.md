# CONSENSUS — feature.compose → Parametric 特征史

## 需求

AI ActionPlan 调用 `extrudeSketchProfileToBrep` / `filletEdgesToBrep` / `linearPatternBodyToBrep`，写入 `ParametricBrepBackendData` 特征链（真特征史），不再仅用 mesh 布尔代理。

## 已确认决策

1. **Domain**：新建 `feature.compose`；拉伸/凸台/法兰/零件/阵列/圆角/建模/text-to-cad 等路由至此；纯布尔/通孔等仍 `mesh.compose`
2. **MVP API**：Pad（矩形/多边形轮廓）→ Pocket → Fillet（edge_indices）→ LinearPattern
3. **特征树**：Host 发 `onParametricBodyHistoryChanged`；geomodeling 页存在则 `setActiveBodyId` + `syncFeaturesFromBody`；验收以场景 Body + history JSON 为准

## 验收标准

- [ ] `feature.compose` 计划可创建含 Sketch+Pad 的 Parametric Body
- [ ] 后续 Pocket/Fillet/Pattern 追加到同一 `$stepId` Body
- [ ] `askClarify` 合法；尺寸不全时停手
- [ ] 几何建模工作区打开时特征树刷新
- [ ] Debug|x64：SDK / Host / GeometricModelingPlugin 编译通过
- [ ] ABI ≥ **1.44.0**（`0x00012C00`）

## 边界

- 不做 Sweep/Loft/Shell/Draft/Revolve（下轮）
- Fillet 不保证自动拓扑边选择；`edges=all` 仅尽力（按 tip 边数 0..N-1）
- 不强制 claim `com.cloudsim.geomodeling` 即可写 Body
