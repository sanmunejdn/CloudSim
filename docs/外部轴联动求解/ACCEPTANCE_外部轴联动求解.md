# ACCEPTANCE：外部轴联动求解

## 整体验收

| # | 标准 | 状态 |
|---|------|------|
| 1 | Dock「外部轴」Tab 可配置地轨并持久化 | 已实现 |
| 2 | 未配置时 ExternalAxisSearch 不写假 rail | 已实现（no-op） |
| 3 | 启用后 Search 写 snapshot + reachable | 已实现（需 URDF 上下文） |
| 4 | MDH 解析雅可比 API `positionJacobianAnalytic` | 已实现 |
| 5 | 联立路径棱柱 dq 按 mm（±50）限幅 | 已实现 |
| 6 | 工程文件可编译 | 已通过（RobotScene + RobotWidget Debug\|x64，2026-07-23） |
| 7 | 文档存 P0；运行时 qe 合成 P_eff，网格不变形 | 已实现 |
| 8 | TCP 拖动可驱动地轨（联立 / 协调候选） | 已实现 |
| 9 | Run/点云播放：臂轨迹插帧 + 外轴 qe 插值，非瞬移 | 已实现（2026-07-23） |

## 子任务完成

- T1–T4：模型/IO/UI/门禁注入 — 完成
- T5–T7：Kinematics / TeachIk / 联立 — 完成
- T8：真搜索 — 完成
- T9：文档与指南 — 完成（含播放插帧修订）
- T10：Run 外轴/轨迹插帧 — 完成

## 手工验证建议

1. 导入带基座的机器人 → 外部轴 Tab → 添加地轨 → 启用 → 存盘重开
2. 轨迹管道加 ExternalAxisSearch：未配置时 ctx.externalAxes 不变
3. 启用后运行管道：出现 jointName 快照，点 reachable 随 IK 结果变化
4. **点云/多 PTP-LINE Run**：整机随点位连续运动；地轨滑条随进度滑动，勿首帧跳到终点
5. 臂几乎不动、仅地轨大行程：段时长应明显长于 0.05s，可见滑移动画
