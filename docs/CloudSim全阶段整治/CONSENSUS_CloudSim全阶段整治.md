# CONSENSUS — CloudSim 全阶段整治

## 需求描述

分阶段完成 CloudSim 目录卫生、静态缺陷审计、Top3 定点修复，并以 Debug|x64 与 Release|x64 双编验收。

## 技术约束

- 外科手术式改动；注释纯中文、聚焦 Why
- 空间相关遵守 `spatial_contract_world_pose.md`
- 空 catch 仅加可观测日志，不改变失败语义
- 不实现未完成的 Mesh 模式算法，仅入口防护

## 验收标准

1. 安全垃圾已清；历史任务 docs 位于 `docs/_archive/`；常读文档与 `docs/README.md` 索引正确
2. `BUG_AUDIT.md` 覆盖 Robot / TrajUI / GeoMesh / CatchIO 四路；非 Top3 仅记录
3. Top3 有明确修复或「证据不足不改」说明；无范围外大重构
4. 触及工程 **Debug|x64 与 Release|x64 均编译成功**
5. 6A 文档（ALIGNMENT/CONSENSUS/DESIGN/TASK/ACCEPTANCE/FINAL/TODO）齐全

## 集成方案

- 文档：`docs/CloudSim全阶段整治/` + `docs/_archive/`
- 代码：仅改 Top3 相关源文件及必要头文件
- 构建：`msbuild <vcxproj> /p:Configuration=Debug|Release /p:Platform=x64`
