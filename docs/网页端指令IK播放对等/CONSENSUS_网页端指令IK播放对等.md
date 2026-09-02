# CONSENSUS_网页端指令IK播放对等

网页 Run/预览与桌面同一套契约（见 `docs/指令IK轨迹重构/CONSENSUS_指令IK轨迹重构.md`）：

- 指令只存 TCP；规划走 IK，禁止 taught 关节短路
- LINE/ARC/PTP 终点折到链种子最近圈；越限则失败并停在该段
- Run：急算前 16 段，其余 `lazyPending`，播放中补算；折圈失败写入 `failed`，执行器停机

实现落点：`planMotionInstruction`（`/api/robot/plan`）+ `HeadlessRobotPlaybackBridge`（`/api/robot/run`）+ 网页 `playback.ts`。
