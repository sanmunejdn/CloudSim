# TODO — 跟随对象框架隔离

1. **本地联调**：全量编过 Host/Widget 后，按 ACCEPTANCE 手工验跟随与装配。
2. **环境**：若 Link 报缺 `GeometryEngine.lib` / `RobotUrdf.lib`，补齐库路径（与本次逻辑无关）。
3. **可选加固**：属性面板对 URDF 隐藏 `follow.*` 行，减少误操作（当前已在提交路径拒绑）。
