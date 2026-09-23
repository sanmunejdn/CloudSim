# InstantMeshesLib 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | InstantMeshes 库工程（见 `.vcxproj`） |
| 职责 | InstantMeshes 库侧编译单元，与 InstantMeshesCore 配套 |

## 2. 边界

- 与 Core 分工以 vcxproj 源清单为准；产品调用优先经 Geometry 封装层

## 3. 验证

Debug|x64 + Release|x64；改后需连带编 InstantMeshesCore 与上游 Geometry 模块。
