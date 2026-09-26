# ALIGNMENT — 自动化测试

## 1. 原始需求

希望能自动验证软件稳定性与功能，分析 bug，并形成可重复的修复闭环。

## 2. 任务边界

| 纳入 | 不纳入本波 |
|------|------------|
| SelfTestRunner 聚合现有 `runSelfTest` | 桌面 Qt UI 自动化（Spy/QTest 全量） |
| CloudSimWeb headless REST/SSE pytest | MuJoCo/动力学仿真测试 |
| 崩溃 MiniDump + soak 内存趋势 | 公共 GitHub Actions runner |
| self-hosted CI + 失败 bundle | 前端 Playwright 全量 E2E（可后补） |
| Debug\|x64 + Release\|x64 双配置 | 改 `OutDir` / 私改产物路径 |

## 3. 现状上下文

- 自检散落：RobotUrdf / GeometryEngine / GeometryAlgorithm / PointCloudAlgorithm / VcgAlgorithms / RobotPathPlanning / GeometryServices::MeshBoolean
- 无统一自检入口、无测试工程、无 CloudSim 自有 CI
- 自动化最佳入口：`CloudSimWeb.exe --port=` + [API_CONTRACT.md](../网页端/全量对等/API_CONTRACT.md)
- 机器人资源 `resource/models` 可能缺失 → 相关用例 skip

## 4. 关键决策（已确认）

1. 统一自检入口 = **独立 SelfTestRunner.exe**（不侵入 CloudSimWeb 链接图）
2. 范围 = **全量**：门禁 + dump + soak + CI + 修复闭环
