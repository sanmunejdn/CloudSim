# ALIGNMENT_网页端

## 原始需求

基于 CloudSim 现有架构，并行开发网页端：Three.js 前端 + 本地独立服务进程；桌面版功能与工程集零回退。

## 项目特性

- 桌面：Qt + OSG，`CloudSim.sln` / `CloudSim.exe`
- 契约：`IDataService` / `IRobotService` / `EventHub` / `.pcp` v4
- 输出：`Directory.Build.props` → `bin\x64d` / `bin\x64`

## 边界确认

| 纳入 | 排除（本期） |
|------|----------------|
| 双 sln、双进程 Web 服务 | 把 Gateway 挂进桌面进程 |
| Headless 文档会话（旁路 API） | 改写桌面 DocumentPage / main 语义 |
| M0 health + 静态页；M1 开工程/场景 | 完整 Ribbon、全插件、跨进程实时协同 |
| Data 权威，浏览器拾取回写 | 同文件静默互盖 |

## 需求理解

网页端是**并行产品面**，不是替换桌面二进制。共享 DLL 可同时加载；编译同 OutDir 须串行。

## 疑问澄清（已决）

1. 双 sln 隔离 — 已确认  
2. 双进程互不影响 — 已确认  
3. 桌面零影响最高优先 — 已确认  
4. 首期 Web 进程可链 OSG DLL，但不创建桌面主窗口 — 已确认  
