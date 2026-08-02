# CONSENSUS：仿真倍率

## 验收标准

1. 下拉预设：`0.1× / 0.25× / 0.5× / 1× / 2× / 5× / 10×`，默认 `1×`
2. `0.5×` 播放更慢、`2×` 更快；`plan.durationSec` 不变
3. 运行中改倍率进度连续
4. WAIT 与外轴插帧同步受倍率影响
5. `RobotScene` + `RobotWidget` Debug|x64 与 Release|x64 编译通过

## 技术方案

- UI：`SimulationCommandWidget` 倍率 `QComboBox` → `playbackRateChanged`
- 桥接：`RobotSimulationController` 启动时与信号同步 `RobotProgramExecutor::setPlaybackRate`
- 核心：`simElapsed += dtWall * rate`，`u = simElapsed / durationSec`
