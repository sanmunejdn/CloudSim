# TODO — 视口拾取重构

- 手工全量回归（ACCEPTANCE 清单）
- FeatureTrajectory / AssemblyMate / 插件改为显式 `beginInteractionSession`（可去掉对 setMeshPickCommittedHandler 的依赖）
- `RobotObjectSelectPolicy` 在 DocumentPage 打开时注入 Controller（替代仅 Passthrough）
- 删除旧 Operation 类文件（逻辑已由 Tool 适配器承载后）
- Headless stub 对齐 Session API（若有编译点）
- Host.vcxproj.filters 中 ViewportInteraction 筛选器节点核对
