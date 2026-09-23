# InstantMeshesCore 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | InstantMeshes 相关静态/中间库（见 `.vcxproj` OutDir） |
| 职责 | InstantMeshes 算法内核薄封装，供 Geometry 链路重网格使用 |

## 2. 边界

- 第三方 InstantMeshes 树不在本指南范围；勿改 vendored 算法正文除非升级依赖
- 调用方多为 GeometryAlgorithm / 相关插件

## 3. 验证

Debug|x64 + Release|x64 编本工程及依赖它的上层模块。
