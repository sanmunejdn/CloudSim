# ACCEPTANCE — 仿真统一内核

跟踪单：BugTracker **#48**

| # | 用例 | 阶段 | 状态 |
|---|------|------|------|
| A1 | 单机 PTP/LINE/WAIT/IF/SET_DO 回归 | P0 | 代码就绪，待手工验 |
| A2 | DI→设备姿态与机器人共用同一 playback timer | P0 | 代码就绪（Player 无独立 QTimer） |
| A3 | Run All 两机并行关节互不覆盖 | P1 | 代码就绪（Run All 按钮） |
| A4 | 两机 DeviceAxis 抢同一设备 → 冲突方 Abort | P1 | 代码就绪（Lease Abort） |
| A5 | 机 DO→设备→Done→机 Wait→继续 | P2 | 代码就绪（下一拍 IO + Done DO） |
| A6 | 统一 Stop 停臂与设备 | P2 | 代码就绪 |
| A7 | 法兰挂载设备统一 Run 无闪裂 | P3 | 框架就绪（需填 mount edges） |
| A8 | SyncBarrier 后对齐进入下一段 | P4 | 代码就绪 |
| A9 | 外置占用实例无程序双写 | P5 | 代码就绪（ControlAuthority） |
| A10 | 碰撞 FreezeLane / StopAll | P5 | 框架就绪（需注入 probe） |
| A11 | Debug\|x64 + Release\|x64 编译通过 | 每阶段 | **通过**（RobotScene + RobotWidget） |

## 编译记录

| 阶段 | Debug | Release | 日期 |
|------|-------|---------|------|
| P0–P5 合并交付 | RobotScene+RobotWidget OK | RobotScene+RobotWidget OK | 2026-10-14 |
