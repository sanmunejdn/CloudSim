# CONSENSUS — 工程筛选器整理

## 1. 需求描述

整理 `CloudSim.sln` 全部工程的 Visual Studio 筛选器：在保留 `inc` / `src` 顶层约定下，按功能点细分，便于 Solution Explorer 导航。

## 2. 已确认决策

| 项 | 选择 |
|----|------|
| Q1 划分策略 | **A** 有子目录则镜像；扁平工程按文件名/职责聚类 |
| Q2 层级 | **A** 功能夹挂在 `inc\` / `src\` 下；**C** Builtins 的 `ops\<Op>` 顶层镜像磁盘 |
| Q3 范围 | **B** 大工程优先 + 小工程轻量/补缺 |
| Q4 缺失 filters | **是**，5 个工程一并补齐 |
| Q5 跨工程头 | **B** 单独 `External`（含 SDK 第三方源） |

## 3. 技术约束

- **只改** `*.vcxproj.filters`（及缺失时新建）；**不改** `.vcxproj`、不搬源文件、不改 `.sln` Solution Folder
- 筛选器条目必须覆盖对应 `.vcxproj` 中的所有编译项（ClInclude / ClCompile / QtMoc / None / …）
- 编码：UTF-8 BOM + CRLF
- Filter `UniqueIdentifier` 使用稳定 GUID（新建时随机，已有可保留）

## 4. 验收标准

1. 优先大工程（Builtins、GeometryAlgorithm、Widget、RobotWidget、Data、RobotScene、CloudSimHost、PointCloudAlgorithm、ProcessFlowPlugin、IndustrialCameraSDK）功能筛选器可见且归类合理
2. 5 个缺失 `.filters` 已补齐
3. 其余 SLN 工程至少有轻量功能细分或磁盘子目录镜像
4. VS 打开后无「筛选器孤儿」；每个 vcxproj 项均有对应 Filter
5. 跨工程 / SDK 引用归入 `External`

## 5. 任务边界

- 不处理 SLN 外工程（HelloAiPlugin、CloudSimPluginHost）
- 不调整物理目录与 include 路径
