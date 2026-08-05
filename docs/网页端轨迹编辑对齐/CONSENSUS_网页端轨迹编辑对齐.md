# CONSENSUS — 网页端轨迹编辑对齐

## 验收标准
1. 工艺模板行：Raw 状态 + 焊缝/涂胶/打磨 + 填充 + 生成
2. 程序/组选择行
3. 左调色板（中文）| 右流水线（序号+中文+启用勾选+删除）
4. 参数区 schema 驱动；底部预览/应用/重置/撤销/重做
5. 模板：下拉 + 保存/加载/删除/导入/导出
6. 新建算子 `enabled:false`；启用后才参与预览/应用
7. 生成 = Raw→LINE（不经流水线）；已应用后禁用生成
8. Host/Gateway Debug|x64 + Release|x64；部署 public-fallback → bin web

## 技术
- Host：`emitRawProgram`、`opPaletteJson`、session `emitDisabled`
- Gateway：`POST /api/trajectory/emit`、`GET /api/trajectory/op-palette`
- 前端：`#robotTrajEdit` 重排 + JS 逻辑
