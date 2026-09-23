# CloudSimBootstrap 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` / `CloudSimWeb.sln` |
| 产物 | 头文件库（API 声明）；实现在 `CloudSimHost.dll` |
| 职责 | 组合根入口：`cloudsimCreateApplicationContext` / `cloudsimCreateHeadlessApplicationContext` |

## 2. 边界

- **负责**：对外导出创建桌面/Headless `ICloudSimContext` 的符号声明
- **不负责**：Host 内部组合、文档页、渲染实现（见 [CloudSimHost](../../Host/CloudSimHost/DEVELOPER_GUIDE.md)）

## 3. 关键入口

- `inc/CloudSimBootstrap.h`
- 桌面 exe 调 `cloudsimCreateApplicationContext`；网页 exe 调 `cloudsimCreateHeadlessApplicationContext`

## 4. 相关文档

- [CloudSimHost](../../Host/CloudSimHost/DEVELOPER_GUIDE.md)
- [网页端 Host 同步](../../features/网页端/Host同步/README.md)
