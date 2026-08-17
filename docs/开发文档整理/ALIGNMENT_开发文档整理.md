# ALIGNMENT — 开发文档整理（按软件模式 / 插件类型）

## 1. 项目上下文

| 项 | 现状 |
|----|------|
| 产品入口 | [`CloudSim/README.md`](../../README.md) → 指向 `docs/README.md` |
| 开发文档真源 | **`CloudSim/docs/`**（常读 + 进行中专题 + `_archive`） |
| 仓库根 `docs/` | 独立 6A/诊断稿（打包、RunLogger、URDF、SDK 目录等），**不是** CloudSim 模块文档主索引 |
| 现有模式文档 | 仅 [`几何建模/`](../几何建模/) 有活跃 hub；工程图 / 工艺流程多在 `_archive`，README 链接有过期路径 |
| 工作区模式（代码） | 内建「主程序」`modeId=""`；插件注册 `com.cloudsim.geomodeling` / `processflow` / `drawing`（顺序：主→几何→工艺→工程图） |
| 插件清单 | 10 个 `plugin.json`：3 个工作区模式插件 + 侧栏/能力插件（几何、点云、PLC、相机、标注、PointNet、HelloAi）+ SDK 工程 |

## 2. 原始需求

整理开发文档，按软件不同模式（主程序、几何建模、工程图、工艺流程）和插件类型组织，方便查找。范围涉及用户点名的 `docs` 与 `CloudSim/README.md`。

## 3. 需求理解

目标是**检索结构**，不是重写业务设计：让新人/改模式/改插件时，从统一导航进到对的 `DEVELOPER_GUIDE` / 活跃 README / 归档专题。

建议交付形态（待确认）：

1. **模式导航**：主程序 / 几何建模 / 工程图 / 工艺流程各有入口页（链接到源码指南 + 活跃文档 + `_archive`）
2. **插件类型导航**：工作区模式插件 vs 侧栏/工具插件 vs SDK/ABI vs AI 域
3. 更新 `CloudSim/docs/README.md` 与 `CloudSim/README.md` 的文档入口表
4. 修正已知断链（如 ProcessFlow README 仍指向已迁入 `_archive` 的路径）

## 4. 边界确认

| 在范围内 | 不在范围内（默认） |
|----------|-------------------|
| `CloudSim/docs` 索引与各模式/插件 hub | 大段重写各模块 `DEVELOPER_GUIDE` 正文 |
| 更新 `CloudSim/README.md` 文档链接 | 改插件运行时行为 / 代码 |
| 断链修复、活跃入口补齐 | 无必要地搬迁 `_archive` 内数百篇 6A 正文 |
| （可选）仓库根 `docs/` 加一句「非 CloudSim 模块索引」说明 | 把根 `docs/` 与 `CloudSim/docs` 合并成一套目录 |

## 5. 疑问澄清（需决策）

见下方「关键决策点」。

## 6. 暂定信息架构（供确认）

```text
CloudSim/docs/
├── README.md                    # 总索引：先「按模式」再「按插件类型」再「常读横切」
├── 主程序/README.md             # 新建：三维主工作区（Widget/Host/Robot/轨迹/网页）
├── 几何建模/                    # 已有，微调交叉链接
├── 工程图/README.md             # 新建活跃入口 → 插件 README + _archive/工程图纸
├── 工艺流程/README.md           # 新建活跃入口 → 插件 README + _archive/工艺流程*
├── 插件/README.md               # 新建：按类型索引全部插件与 SDK
└── _archive/                    # 保持物理位置不动（默认）
```

插件类型草案：

| 类型 | 成员 |
|------|------|
| 工作区模式 | GeometricModeling / ProcessFlow / EngineeringDrawing |
| 侧栏工具 | GeometryPlugin / PointCloudPlugin / PlcComm* / IndustrialCamera* / Labeling* |
| AI | CloudSimAiSDK / AiWidget / PointNet / HelloAi |
| 轨迹 SDK | CloudSimMeshTrajectorySDK（主程序侧，非独立侧栏插件） |
| 契约 SDK | CloudSimPluginSDK |

仓库根 `docs/`：保留任务稿；在根加极短 `README.md` 指向 `CloudSim/docs`（可选）。
