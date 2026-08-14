# CONSENSUS — HostOptimization

## 需求

路径 **B**：框架主线优先，Host 卫生穿插；物理拆分仅 AiHost 可选。

## 技术方案

1. **契约**：新查询能力进 `IDataService`；UI 新代码禁 `backend()`；白名单只减不增。
2. **Host 卫生**：`inc|source` 按 `import/project/robot/headless/follow/core` 分目录；`AdditionalIncludeDirectories` 保扁平 `#include`。
3. **DocumentHost**：`DocumentProjectSidecar` + `DocumentFollowState` 持有旁路表与 Follow 脏集。
4. **Headless**：桌面/Web ops 以 Host Headless* / 已有 Facade 为共路真源；差异记入对齐表。
5. **Controller**：同工程内按域拆 `.cpp`，类仍在 RobotWidget。

## 验收标准

- Wave1：`INTERFACE_CATALOG` + `BACKEND_CALLSITE_INVENTORY` 齐全
- Wave2+：Debug|x64 与 Release|x64 相关工程编译通过
- `backend()` 外部调用点有清单且至少完成一类上提迁移
- DocumentHost 公开状态字段迁入 Sidecar/FollowState
- Controller 由单文件变为多编译单元且行为不变

## 技术约束

- 遵守 `spatial_contract_world_pose.md`
- 插件仍只链 PluginSDK
- 不改 `Directory.Build.props` 输出路径
