# TODO：外部轴联动求解

## 待办

1. **变位机 UI/求解**：`RobotExternalAxisKind::Turntable` 已预留，未实现
2. **程序导出外轴字段**：品牌导出是否写 E1 等需产品确认
3. **URDF 真棱柱进 q 向量**：当前地轨为基座等价平移；若 URDF 含 prismatic 且需进聚合关节向量，需扩展绑定管线
4. **雅可比数值对比单测工程**：API 已提供，可加独立 test 工程断言误差 &lt; 1e-5
5. **地轨速度可配置**：播放时长目前按固定 ~250 mm/s 梯形估时；后续可挂到外轴配置或指令 speed

## 已修复（轴控制外轴滑条）

- 「轴控制」页对已启用外轴显示同风格滑条/数值框
- 拖动按 `P * Trans(q·axis)` 更新基座并重贴 FK（文档存 P0，运行时 `P_eff`）
- 改外部轴配置时同步轴控 UI

## 已修复（PTP 外轴）

- Host `planMotionInstruction` 注入 `setExternalAxes`
- PlanResult / DTO 回传 `externalAxisQ`，指令扩展写回 `context.externalAxisQMm` / `Dir`
- Worker / 可行性探测同步外轴；改配置清计划缓存

## 已修复（运行求解链路整理）

统一契约：
1. **求解**：启用外轴时网格无解则失败，禁止静默臂-only
2. **规划**：Preview/Run 直调本地 `Controller::plan`（完整 PlanResult，含 `hasExternalAxisQ`/`durationSec`）
3. **应用**：文档存 `externalAxisQMm`（P0 不烘焙）；FK 时 `P_eff = P0 * Trans(q·axis)`；预览/播放写 qe 再贴关节

## 已修复（点云/Run 播放插帧，2026-07-23）

此前 Run 时机器人「一瞬间就位」的根因：

1. 播放每 tick 把 plan 的目标 `qe` **直接写入**基座（外轴瞬移）
2. 急算/缓存写入时 **清空** `jointTrajectoryRad`，LINE/点云多样本无法插帧
3. 臂 Δq≈0、地轨行程大时，`durationSec` 仅按关节估会接近下限，看起来像瞬移

修复要点：

| 项 | 行为 |
|----|------|
| 外轴插值 | `applyExternalAxisFromPlan(..., progress01, segmentStartQMm)`；段切换记下起点 qe |
| 进度源 | `RobotProgramExecutor::motionSegmentProgress01()` |
| 轨迹保留 | 急算 / `commitPlaybackPlan` / lookahead **不再** `clear` 轨迹 |
| 时长 | 按 \|Δqe\| 以 ~250 mm/s 梯形抬高 `durationSec`（规划与 commit 双路径） |

## 配置提示

- 地轨轴方向默认基座系 X；按现场改 Axis X/Y/Z
- 行程单位 mm；联立步长硬限 ±50 mm
- **验证 Run**：启用地轨 → 点云/多点程序运行；臂与滑轨应连续运动（非瞬移）；轴控滑条跟 plan 的 qe；失败时报「外轴联立未找到可行解」
