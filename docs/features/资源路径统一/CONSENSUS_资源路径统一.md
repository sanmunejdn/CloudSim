# CONSENSUS — 资源路径统一

**决策日期**：2026-10-14  
**方案**：A + 构建自动同步 models + 删除 `bin\resource` 下脚本/JSON 副本（改由构建生成）

## 1. 需求与验收

### 开发口诀（唯一约定）

| 改什么 | 权威路径 |
|--------|----------|
| 脚本 / JSON（控制器、导出、轨迹、特征等） | 各模块 `CloudSim\src\...\resource\` |
| 设备模型（URDF/网格） | `bin\resource\models\` |
| 运行时加载 | `{exe}\resource\`（`bin\x64` / `bin\x64d`） |

### 验收标准

1. 不存在目录 `CloudSim\resource\`。
2. `ControllerPython` / `controllers` 位于 `CloudSim\src\UI\RobotWidget\resource\`，与 `ExportPython` 同树。
3. 构建 RobotWidget / CloudSim 后，OutDir 含 `resource\Python\ControllerPython`、`resource\controllers`、`resource\Python\ExportPython`、`resource\trajectory`（既有逻辑）、以及自 `bin\resource\models` 同步的 `resource\models`。
4. `bin\resource\` 顶层**仅**保留 `models`（无 `Python`、`trajectory` 副本）。
5. `ExternalControllerDialog` 开发态回退指向 RobotWidget 源路径。
6. Debug\|x64 与 Release\|x64 均能编译；相关 PostBuild 无失败。

## 2. 技术方案

- **迁移**：`CloudSim\resource\**` → `CloudSim\src\UI\RobotWidget\resource\`（合并进已有 `resource\Python`）。
- **拷贝**：`RobotWidget.vcxproj` 的 Controller/controllers 源改为 `$(ProjectDir)resource\...`。
- **models 同步**：`CloudSim.vcxproj` 增加 PostBuild Target，将 `$(CloudSimRepoRoot)bin\resource\models` 同步到 `$(OutDir)resource\models`（SkipUnchanged / robocopy 等价）。
- **清理**：删除 `bin\resource\Python`、`bin\resource\trajectory`；打包仍优先 OutDir，缺省时用模块源 fallback（已有 trajectory/ExportPython；补 Controller 若需要）。
- **文档**：外置控制器 README/验收路径改为 RobotWidget 下。

## 3. 边界

- **不改**运行时相对布局（仍是 `resource/models` 等）。
- **不迁** models 出 `bin\resource`。
- **不**把 trajectory/feature/ExportPython 再集中到单一总仓（方案 C 不做）。
- **只编 WebGateway、不编 CloudSim 时不同步 models**（`SyncSharedModels` 仅挂桌面 `CloudSim.vcxproj`）。
- 历史外置控制器文档旧绝对路径已清理；`Setup\packaging\dist` 用 `collect-deps -Clean` 刷新。

## 4. 约束

- 遵守 `OutDir` / `CloudSimBinDir`；双配置构建验证。
- 注释：中文、只写 Why。
