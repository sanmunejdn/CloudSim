# TASK — 工程图模块清理

## 依赖图

```mermaid
flowchart LR
  T1[T1 删插件/sln] --> T3[T3 Host/SDK ABI]
  T2[T2 删 HLR] --> T3
  T3 --> T4[T4 文档帮助]
  T4 --> T5[T5 双配置编译]
  T5 --> T6[T6 验收文档]
```

| ID | 任务 | 验收 |
|----|------|------|
| T1 | 删插件 + sln + organize 脚本 | 目录与工程条目不存在 |
| T2 | 删 GeometryAlgorithm Drawing/HLR | vcxproj 无对应项且可编译 |
| T3 | SDK/Host/Web Bridge + 版本抬升 | grep 无活跃符号 |
| T4 | README/features/help | 无「可用工程图」表述 |
| T5 | Debug+Release 依赖链编译 | 退出码 0 |
| T6 | ACCEPTANCE/FINAL/TODO | 文档齐全 |
