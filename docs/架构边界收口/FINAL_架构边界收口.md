# FINAL：架构边界收口

> 日期：2026-07-21  
> 状态：**本轮闭环完成**（Sprint A–H）  
> 范围：Host/UI 边界硬化 — 公开契约去 OSG、Host 解耦 RobotWidget、DocumentPage FK 存储 Mat4

## 1. 目标回顾

将 CloudSim「Host 收口」从半完成状态推进到可维护边界：

- UI / RobotWidget 新代码经 `doc->data()` / `render()` / `robot()`，不再扩散 `BackendDataManager` / OSG 头
- Host 不再反向依赖 `RobotWidget.lib`
- 公开运动学/视口契约使用 `core::Mat4` / `Vec3`，OSG 仅在 OsgWidget / 适配层转换

## 2. Sprint 总览

| Sprint | 主题 | 验收 |
|--------|------|------|
| A | PluginHost 入 Host；Follow 契约；删 Host/osg 平行副本 | [ACCEPTANCE](ACCEPTANCE_架构边界收口.md) |
| B | 基座/bind Mat4；Host 不写 kinematics 根；视口 toolbar→render() | [SprintB](ACCEPTANCE_架构边界收口_SprintB.md) |
| C | RobotProgramStore→RobotScene；Host 无 RobotWidget.lib；ViewHost Mat4 | [SprintC](ACCEPTANCE_架构边界收口_SprintC.md) |
| D | RobotOsgUiTypes 去 osg | [SprintD](ACCEPTANCE_架构边界收口_SprintD.md) |
| E | mesh 高亮 / 拟合面 → Vec3 | [SprintE](ACCEPTANCE_架构边界收口_SprintE.md) |
| F | IRobotSimulationDocument 去 osg；OSG 切片独立头 | [SprintF](ACCEPTANCE_架构边界收口_SprintF.md) |
| G | IRobotBackendPoseSink Mat4 | [SprintG](ACCEPTANCE_架构边界收口_SprintG.md) |
| H | DocumentPage FK 存储 Mat4；backend() 策略闭环；Assess | [SprintH](ACCEPTANCE_架构边界收口_SprintH.md) |

## 3. 质量评估

| 维度 | 结论 |
|------|------|
| 架构对齐 | 与 `ARCHITECTURE_SUMMARY.md` §11 一致；§11.1 新代码边界已更新 |
| 编译 | Debug\|x64：`RobotScene` + `CloudSimHost` + `RobotWidget` + `Widget` 通过（2026-07-21） |
| 契约稳定性 | Core/Host 对外接口以 Mat4/DTO 为主；vtable 追加遵守既有约定 |
| 技术债务 | 见 [TODO](TODO_架构边界收口.md)；已显式列为长期项，不阻塞本轮交付 |
| 过度设计 | 无；按 Sprint 切片，未一次性删除 `backend()` 或迁 Controller |

## 4. 关键边界（交付态）

```
App / Widget / RobotWidget
    │  data() / render() / robot() / events()
    ▼
CloudSimHost (DocumentHost + OsgWidget 编译单元)
    │  适配器内 BackendDataManager / OSG
    ▼
Data / RobotScene / OsgWidgetCore / …
```

- **OSG 真源**：仅 `src/UI/Widget/source/OsgWidget*`（由 Host 工程编译）
- **FK 绑定真源**：DocumentPage 内存为 `core::Mat4`；OSG 切片经 `RobotPerLinkKinematicsSliceOsg.h`
- **Mat4 布局**：列主序 `index = c*4+r`（与 Core / IRenderView 一致）

## 5. 手工回归（仍建议执行）

- 打开/保存含机器人 `.pcp`
- URDF 导入 → per-link FK / 回放
- TCP 示教、视口左右栏、Follow
- mesh 三角高亮 / 拟合面预览

## 6. 结论

**架构边界收口本轮目标已达成并可冻结。** 后续工作按 TODO 分项推进，不再作为同一「边界收口」程序阻塞项。
