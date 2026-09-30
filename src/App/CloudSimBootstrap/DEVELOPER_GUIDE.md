# CloudSimBootstrap 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` / `CloudSimWeb.sln` |
| 产物 | **仅头文件**（Utility 工程，无 `.lib`） |
| 职责 | 声明组合根 API；符号由 `CloudSimHost.dll` / `CloudSimHostHeadless.dll` 导出 |

## 2. 边界

- **负责**：`CloudSimBootstrap.h` 等公开声明（`CLOUDSIM_HOST_EXPORT`）
- **不负责**：ApplicationContext 实现、Registry 注入、文档页、渲染（见 [CloudSimHost](../../Host/CloudSimHost/DEVELOPER_GUIDE.md)）
- **禁止**：在本工程再编译 `CloudSimApplicationContext.cpp`（与 Host 双份实现会导致 C4273 / 半注入分叉）

## 3. 关键入口

- `inc/CloudSimBootstrap.h`
- 桌面 exe 调 `cloudsimCreateApplicationContext`；网页 exe 调 `cloudsimCreateHeadlessApplicationContext`（链接对应 Host DLL）

## 4. 相关文档

- [CloudSimHost](../../Host/CloudSimHost/DEVELOPER_GUIDE.md)
- [网页端 Host 同步](../../features/网页端/Host同步/README.md)
