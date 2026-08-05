# TODO：3D 视口线框显示模式

## 待你本地确认

1. **手工点验**：导入 BRep → 开/关线框；Mesh 对比；线框开着再导入 BRep（见 ACCEPTANCE）
2. 若需完整 exe程序：按依赖编 `CloudSim` 入口工程（本次已编过 `CloudSimHost` + `BackendVisual`）

## 非阻塞后续（可选）

- 独立「着色+边」显示模式（本次明确不做）
- Mesh 线框改为 feature-edge（本次保持三角 LINE）
- Phase2 失败时的用户提示（当前兜底：不隐藏填充）

## 无缺配置

无新增环境变量 / 插件 / 第三方依赖。
