# 轨迹参数 JSON 资源

源码目录：`CloudSim/src/Robot/RobotScene/resource/trajectory/`（**不是** C++ 编译自动生成）。

运行时目录：`bin/x64/` 或 `bin/x64d/` 下的 `resource/trajectory/`。由 **CloudSim** 或 **RobotScene** 的 PostBuild 首次拷贝（不覆盖已有文件）。

若源码目录缺少 `ops/*.json`，运行：

```text
python CloudSim/tools/gen_trajectory_json.py   # ops/ 程序级块 + CommonScope
python CloudSim/tools/gen_atomic_ops.py        # ops/ 原子块四件套 + JSON
```

勿再生成 `resource/trajectory/raw/` 或 `RawTrajectoryRecipes.json`。

## 目录结构

| 路径 | 说明 |
|------|------|
| `ops/<Name>.json` | 原子算法块：schema + defaults，由 `<Name>OpConfig` 加载 |
| `CommonScope.json` | 作用域公共字段（scope.kind / groupId / pointFrom / pointTo） |
| `ProcessFlowPresets.json` | 工艺预设；`pipeline` 为原子块列表（如 Resample → OffsetAlongNormal → …） |

## 执行约定

- **Ingress**：`ingressUnifiedFromProgram` / `ingressUnifiedFromRaw`（CAD 导入时 `importRawPathToTrajectory`）
- **管道**：`TrajectoryPipelineEngine` 在 `UnifiedTrajectory` 上重放 `ITrajectoryOp::processPath`
- **Egress**：`egressUnifiedMergeIntoProgram` / `egressUnifiedToProgram`

## 新增算法块 checklist

1. `TrajectoryAlgorithmBuiltins/ops/<Name>/<Name>Op.h/cpp` — 实现 `processPath`
2. `<Name>OpConfig.h/cpp` — JSON 加载与 C++ 兜底
3. `<Name>OpParamAccess.h/cpp` — 参数读写
4. `resource/trajectory/ops/<Name>.json` — 外置配置
5. 在 `TrajectoryOpBuiltinsRegister.cpp` 注册 Config + ParamAccess

## 兜底顺序

用户 JSON → `*.defaults.json` → `*Op::paramFields()` / `makeDefaultDescriptor()`

## 已移除（2026-06 管道统一）

| 移除项 | 替代 |
|--------|------|
| `RawTrajectoryRecipes.json` | `ProcessFlowPresets.json` 原子 `pipeline` |
| `RecipeWeld` / `RecipeGlue` / `RecipeGrind` 复合 kind | 预设展开为 Resample/Offset/Approach/Retract 等 |
| `buildPreviewPoseQueryChain` / Pose 预览链 | `TrajectoryPipelineEngine::executeFull` |
| `RawTrajectoryRecipeLoader` | 已删除 |
