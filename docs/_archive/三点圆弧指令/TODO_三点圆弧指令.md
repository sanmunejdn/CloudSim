# TODO：三点圆弧指令（CIRC / ARC）

- [x] ARC 示教 CSV 短路导致无圆弧：`shouldUseTaughtJointCsv` 对 ARC 恒 false
- [x] PlanResultCache 保留 ArcPlanner/LinePlanner 的 `jointTrajectoryRad`
- [x] 指令树摘要显示途经点与终点
- [ ] 品牌导出：ABB `MoveC`、FANUC circular、ROKAE/AIR/INOVANCE 对应指令
- [ ] 独立 ARC 图标（现复用 Line）
- [ ] Via 姿态分段 Slerp（当前仅 Start→End）
- [ ] 属性面板「捕获当前为 Via」按钮
- [ ] 本地 x64 完整链接与示教/Run 手工验收签字
- [ ] 与 LINE corner blend「圆弧过渡」区分文档（仍见 `docs/机器人指令执行/TODO`）
