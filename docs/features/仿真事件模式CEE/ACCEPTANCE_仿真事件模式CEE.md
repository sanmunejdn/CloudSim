# ACCEPTANCE — 仿真事件模式（薄 CEE）

| ID | 用例 | 优先级 | 状态 |
|----|------|--------|------|
| A1 | TimeBased 单机 PTP/WAIT/SET_DO | P0 | 代理门闩通过（TimeBased 路径未改；双配置编译） |
| A2 | EventBased DI↑ → goNamedPose；切回 TimeBased Driver 可用 | P0 | 代理门闩通过（CEE 自测边沿 + Driver setEnabled） |
| A3 | 机 DO→wire→设备 DI↑→CEE→Done→机 Wait | P1 | 代理门闩通过（下一拍 IoPhase + CEE 挂点） |
| A4 | EventBased + 外置控制器 Run 拒绝 | P0 | 代理门闩通过（onEventBasedSimulationStart 校验） |
| A5 | Run 中改模式拒绝 | P0 | 代理门闩通过（setRunMode 会话守卫） |
| A6 | 保存/打开 ceeRules + mode | P0 | 代理门闩通过（MainWindowProjectIo 读写） |
| B1 | RobotScene/RobotWidget/Widget Debug\|x64 | P0 | 通过 |
| B2 | 同上 Release\|x64 | P0 | 通过 |

说明：完整三维 UI 点验仍建议人工复扫 A1–A6；上表为阶段 B 开工代理门闩（自测 + 代码路径 + 双配置编译）。
