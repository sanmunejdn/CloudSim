# ALIGNMENT — 架构缺陷分级修复

## 原始需求

按严重/高/中三级修复 CloudSim 架构缺陷：P0 Host 上帝 DLL 治理、P1 正确性类缺陷、P2 中级方向文档。

## 边界确认

- 一期：物理迁移借编源码到 Host，不拆独立 DLL
- 二期（本轮不做）：评估拆 OsgWidget.dll / PluginHost.dll
- Bug 跟踪：#3

## 需求理解

- Host 从 `src/UI/Widget` 与 `src/UI/CloudSimPluginHost` 借编源码，物理位置与编译归属分离
- 双 Host 靠 allowlist 脚本防漂移
- `backend()` 穿透需棘轮强制

## 假设

- 迁移后 include 路径通过 Host `inc/` 与 `AdditionalIncludeDirectories` 覆盖
- `CloudSimPluginHost.vcxproj` 独立参考工程可从 sln 移除
