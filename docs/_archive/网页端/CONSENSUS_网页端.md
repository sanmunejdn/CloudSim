# CONSENSUS_网页端

## 需求与验收

| 项 | 标准 |
|----|------|
| 桌面零回退 | `CloudSim.sln` 工程集/Startup/`main` 业务流不变；共享层只旁路追加 |
| 双 sln | `CloudSimWeb.sln` 仅含 Web 依赖；桌面 sln 不引入 Web 工程 |
| 双进程 | `CloudSim.exe` 与 `CloudSimWeb.exe` 独立；桌面不监听 Web 端口 |
| M0 | `/api/health` 返回 `role=web` + pid；静态页可开 |
| M1 | 打开 `.pcp`/project.json；objects + mesh；Three.js 姿态与桌面同文件一致（文件级） |

## 技术方案

- `cloudsimCreateHeadlessApplicationContext` + `createHeadlessDocumentHost`（Host 追加）
- `CloudSimWebGateway`：cpp-httplib（HTTP + WS）；默认 `127.0.0.1:8787`
- 前端：Vite + React + TS + Three.js → `bin\x64*\web\`
- 位姿权威：进程内 Data；Three.js 视图缓存

## 技术约束

- OutDir 沿用 `Directory.Build.props`，不新增 Configuration
- Web 不链 `Widget` / `RobotWidget` / `AiWidget`
- Gateway IO 线程投递到 Web 进程 Qt 主线程再调契约
- `.pcp` 顾问锁：共享读 / 独占写；冲突返回明确错误

## 任务边界

本期交付 M0–M1；M2–M7 / H2 按 TASK 排期，不阻塞骨架验收。
