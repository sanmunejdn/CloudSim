# 指令 / IK / 轨迹四层

1. **Instruction（持久化）**：仅 TCP（pose/euler + 变换扩展）+ 运动参数 + 轴配置；禁止关节 CSV。存储位姿恒为 **基座（P0）工具原点**；User 系仅显示层。
2. **PlanRequest / IkRequest**：临时；`IkSeedPolicy` = `FromCurrentPose` | `FromInstruction`；种子由 UI 解析后注入 `prepareInstructionIkContext`。求解侧 `IkSolveRequest` / `IkSolveResult`（含 `elapsedMs` / `candidatesTried` / `bestPartialQ`）；`IkSolveMetrics` 只进诊断日志（不写 plan.summary，避免被当失败原因误判 Budget），Budget 类失败时附 Run 警告；失败原因优先取 planner `err`。
3. **IK**：
   - 纯臂：`Controller` → `IkCandidatePipeline`（`IkSeedProvider` → `PoseIkSolverRegistry` 解析/DLS → `IkRankAndFilter`）
   - 外轴：`IkSolveSession` → TeachIk（`externalCoupleMode`；联立仍 ≤2 全平移，否则网格；外轴激活时跳过解析入池）
   - LINE/ARC 样点失败时**局部加密**（段内 0.25/0.5/0.75）再全局加倍
   - `testdata/ik_golden/*.json` + `runIkGoldenSelfTest` 校验肘/支约定（可按包名加真机位姿）
4. **Ruckig（PTP）**：IK 成功后生成 `jointTrajectoryRad`；LINE/ARC 笛卡尔采样。

UI 只组种子并调用 `plan`；连续性/构型门以 Planner `IkSelectionPolicy` 为准。

机型旁路：`…/<pkg>/ik_profile.json`（见 `RobotUrdf` DEVELOPER_GUIDE §`UrdfIkMachineProfile`）。

已知未接线 / 后续：`seedStrategyFlags` 未驱动 SeedProvider；外轴+解析目标换算后入池；FailKind 个别路径仍 `classifyIkFailMessage`；anytime 字段上层未系统消费。模块细则见 [`RobotScene/DEVELOPER_GUIDE.md`](../../../src/Robot/RobotScene/DEVELOPER_GUIDE.md) §3.3 / §5.5、[`RobotUrdf/DEVELOPER_GUIDE.md`](../../../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md) §3。

详见 `docs/ARCHIVE_ZIP_LOCATION.txt`。
