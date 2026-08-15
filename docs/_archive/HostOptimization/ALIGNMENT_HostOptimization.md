# ALIGNMENT — HostOptimization（路径 B）

## 原始需求

全面整理 Host 接口；按功能逻辑拆分；框架评估后确认：**主线（backend 收口 / Headless 共路 / Controller 切片）+ Host 卫生穿插**。不拆 Plugin/Osg DLL；AiHost / Web H2 仅按需。

## 边界

| 做 | 不做 |
|----|------|
| 接口目录、backend 调用清单 | PluginHost / Osg 物理拆 DLL |
| 同 DLL 子目录分层（不改导出符号名） | Controller 整包迁 Host |
| DocumentHost 旁路/Follow 状态外提 | 无诉求时拆 AiHost |
| `IDataService` 可安全上提的查询 API | 改 OutDir / 安装布局 |
| Headless 共路对齐文档与可落地薄共享 | HTTP 迁入 Host |
| Controller 按域拆编译单元 | |

## 验收（Wave 级）

见 [TASK_HostOptimization.md](TASK_HostOptimization.md)。
