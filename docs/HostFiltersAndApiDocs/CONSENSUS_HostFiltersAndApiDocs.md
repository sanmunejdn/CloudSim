# CONSENSUS — Host 筛选器整理与 API 文档

## 需求与验收

| # | 验收标准 | 验证 |
|---|----------|------|
| 1 | `vcxproj` 与 `filters` 的 ClInclude/ClCompile/QtMoc 集合一致 | 脚本 diff 空差集 |
| 2 | 存在 `inc\io` / `src\io`；IO 源与 QtMoc 归入该域 | 打开 filters 可见 |
| 3 | DesignParts Catalog/Handler 归入 Ai Catalog/Domains | 同上 |
| 4 | `DEVELOPER_GUIDE.md` 含 §10 全量 API（Core 三件套方法级 + Host 编排 + Headless + IO） | 文档目录 |
| 5 | 过时表述已改：指令属性已接线；`mergeRobotKinematicsIntoProjectRoot` 仍存在 | 文内检索 |

## 技术方案

- 由 vcxproj 再生 filters，保留既有 Filter UniqueIdentifier。
- DEVELOPER_GUIDE：§2 目录补 `io/`；§4 修正；新增 §10；原 FAQ → §11。

## 边界

仅文档 + filters；不编译（无 C++ 逻辑变更）。
