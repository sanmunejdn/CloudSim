# TODO：架构边界收口（后续）

> 本轮 Sprint A–H 已闭环。下列为**明确延期**项，按需立项，勿与本轮验收混为一谈。

## 1. 代码 / 架构（长期）

| 优先级 | 项 | 说明 | 建议入口 |
|--------|----|------|----------|
| P1 | 去掉 `DocumentPage::backend()` 穿透 | 需把运动学/mesh 所需 `BackendDataManager` API 继续上提到 `IDataService` 或专用 Host 服务后再删 | `IDataService`、`RobotSceneKinematics`、`DocumentPage.cpp` |
| P2 | 关节句柄去 `MatrixTransform*` | per-link 已可不依赖；旧关节树路径仍用场景节点 | `HierarchicalRobotInstance`、`applyRobotJointLocalMatrix` |
| P2 | `RobotSimulationController` → Host | 仿真编排下沉，缩小 RobotWidget 对引擎直连 | `RobotWidget/source/RobotSimulationController.cpp` |
| P3 | IRenderView 全面替代 OsgWidget 直引 | 阶段 3.3–3.4；主窗口/工具栏部分已走 `render().widget()` | `ARCHITECTURE_SUMMARY.md` §11 |
| P3 | Widget 层 OSG include 清零 | 与上项绑定；TCP/截面等仍在 OsgWidget 内部合理 | `Widget/inc`、`OsgWidget*` |

## 2. 配置 / 环境

| 项 | 状态 | 操作 |
|----|------|------|
| 构建命令 | 已知可用 | `MSBuild CloudSim.sln /t:RobotScene;CloudSimHost;RobotWidget;Widget /p:Configuration=Debug /p:Platform=x64`（优先 `.sln`，避免单独 `.vcxproj` OutDir 问题） |
| Python 品牌导出脚本 | 与本任务无关 | 见 `docs/机器人程序品牌导出/` |
| API Key / .env | 无本任务需求 | — |

## 3. 手工验证清单（未自动化）

- [ ] 机器人工程打开/保存后关节角与基座位姿正确
- [ ] URDF 导入后连杆世界矩阵与 Follow 附着
- [ ] TCP 示教拖拽与法兰局部矩阵
- [ ] 视口 L/R 侧栏切换
- [ ] mesh 三角高亮 / 拟合面预览

## 4. 文档索引

| 文档 | 用途 |
|------|------|
| [FINAL](FINAL_架构边界收口.md) | 本轮总结 |
| [ACCEPTANCE Sprint H](ACCEPTANCE_架构边界收口_SprintH.md) | 最近一次验收 |
| `ARCHITECTURE_SUMMARY.md` §11 | 演进状态真源 |
| `Widget/DEVELOPER_GUIDE.md` | `backend()` 白名单与 Mat4 存储约定 |

## 5. 需要你拍板时

1. **是否立项去掉 `backend()`**：工作量大，需先列 `IDataService` 缺口清单。  
2. **Controller 迁 Host**：是否与下一代仿真编排一起做。  
3. **手工回归**：你本地跑完 §3 后，可把勾选结果回写到本文件。
