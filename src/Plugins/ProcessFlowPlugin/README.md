# ProcessFlowPlugin 说明文档

CloudSim 动态插件：在三维场景之外提供 **工艺流程图编辑 + DES（离散事件）仿真**，用于评估产线节拍、缓冲、调度策略与瓶颈。

| 项 | 值 |
|----|-----|
| 插件 ID | `com.cloudsim.processflow` |
| 显示名 | Process Flow / 菜单「工艺流程」 |
| 版本 | `1.3.1`（见 `plugin.json`） |
| 最低宿主 | `1.20.0` |
| 库文件 | `ProcessFlowPlugin.dll` |

---

## 界面布局

进入工艺流程后，中央三维视图被流程页替换，左右侧栏切换为流程专用面板：

| 区域 | 内容 |
|------|------|
| **菜单「工艺流程」** | 进入 / 返回三维 / 运行仿真 / 停止仿真 |
| **中央** | 顶栏工具 + 流程图画布 |
| **左侧** | 节点库 + 选中节点属性 |
| **右侧** | 仿真面板（JobSet + 报表）与 **AI 助手** 叠放（tabify）；进入流程时仅保留 AI 页签 |

顶栏工具：连线模式、显示网格、删除选中、自动排版、适应窗口、导出流程 JSON。

---

## 功能概览

### 节点类型

开始、工位、缓冲、仓库、输送、装配、检测、结束。

常见属性（按节点类型）：

- 节拍 / 加工时间、库存或容量、换型时间、优先级
- 批量（batch）、装配所需输入数、检测报废率
- MTBF / MTTR（忙时累计触发故障与修复）
- 机器类节点可存 `binding`（当前仿真走空执行器，绑定为预留）

### 画布编辑

- 从节点库拖放或双击添加节点
- 连线模式串起路径；支持缩放、平移、删节点/边、边标签、网格
- **自动排版**：按拓扑分层从左到右整理节点位置

### JobSet（工艺模板）

- 可维护多套工序模板（机器节点、加工/换型/优先级）
- **从路径生成**：沿 start→end 自动抽取机器工序
- 若 JobSet 为空，运行仿真时会按路径自动生成 `auto` 模板

### AI 助手（process.flow）

进入工艺流程后，右侧可切换到 **AI 助手**。域下拉选「Process flow」或 Auto，用自然语言描述产线，例如：

> 生成三条工位流水线，缓冲容量 20，检测报废率 2%，用 LPT 仿真 1 小时

助手经 Host 桥接（`IProcessFlowAiBridge`）整图写入画布并同步跑 DES，确认后聊天返回 Makespan/吞吐/瓶颈等摘要。复杂场景可启用 remote LLM；短口语由规则模板兜底。

详见 [`docs/工艺流程AI助手/`](../../../docs/工艺流程AI助手/)。

### DES 仿真与调度

- 离散事件引擎（`DesEngine`），工件按开始节点节拍到达（缺省间隔 30s）
- 默认仿真时长 3600s（报表面板可改）
- 调度策略：**FIFO / SPT / LPT / EDD / CR**（同优先级再 FIFO）
- 支持：**批量凑批**、**装配汇合**、**MTBF/MTTR**、**检测报废**、缓冲/仓库容量阻塞
- **对比全部策略**：并排汇总指标（非多张甘特并排）

### 结果与回放

- 摘要：完成/报废/释放、Makespan、吞吐、WIP、瓶颈等
- 机器利用率表
- 独立对话框：甘特图、操作 Trace 表、策略对比
- 画布回放：忙闲高亮 + token 沿路径移动
- 可导出仿真 JSON / CSV；流程本身可导出 JSON
- 保存工程时写入文档键 `processFlow`（图 + 内嵌 jobSet）

---

## 典型使用流程

1. 打开或新建文档（必须有活动文档）。
2. 菜单 **工艺流程 → 进入工艺流程**。
3. 左侧添加节点，打开 **连线模式** 连成 `开始 → … → 结束`。
4. 选中节点，在属性面板设置节拍、容量、批量/装配/MTBF 等。
5. （可选）右侧 JobSet：**从路径生成** 或手工改工序时间。
6. 设置仿真时长与调度策略 → **运行仿真**（菜单或右侧）。
7. 查看摘要与机器表；打开甘特 / Trace / 策略对比。
8. 使用 **回放** 与时间滑条观察画布状态。
9. 需要时导出结果或流程 JSON；保存工程以持久化流程图。
10. **返回三维场景**（若仿真仍在运行会先停止）。

---

## 建模约束（使用前须知）

- 图中至少有一个 **开始** 与一个 **结束**，且 start→end **存在通路**。
- 路径上至少有一个机器类节点（工位 / 检测 / 装配 / 输送）。
- 批量、装配、MTBF、报废等高级逻辑以 **节点属性** 为主；JobSet 表格侧重工序时间与优先级。
- 修改画布会 **停止仿真并清空当前结果**。
- 空图不会写入工程的 `processFlow` 字段。
- 机器人绑定、CP-SAT/RL 调度器等为预留接口，当前为纯数学 DES。

> 注意：RobotWidget 中的轨迹「工艺预设」与本插件无关，请勿混用概念。

---

## 构建与部署

### 依赖

仅链接 `CloudSimPluginSDK`（不依赖 OSG / Widget 静态库）。宿主侧通过 `IPluginHostContext` 提供中央页、侧栏、菜单、文档读写与 **工艺流程 AI 桥接**（1.20.0+）。

### 生成

在 `CloudSim.sln` 中生成 **ProcessFlowPlugin**（Debug|x64 / Release|x64）。

### 运行时目录

```text
bin/x64d/   或   bin/x64/
  CloudSimPluginSDK.dll
  plugins/com.cloudsim.processflow/
    plugin.json
    ProcessFlowPlugin.dll
```

宿主扫描 `plugins/*/plugin.json` 并加载对应 `library`。将 `enabled` 设为 `false` 可禁用本插件。

---

## 源码结构（开发指引）

| 层级 | 主要文件 |
|------|----------|
| 插件壳 | `inc/ProcessFlowPlugin.h`, `source/ProcessFlowPlugin.cpp`, `plugin.json` |
| UI | `ProcessFlowPageWidget`, `ProcessFlowCanvasWidget`, `ProcessFlowPaletteWidget`, `ProcessFlowPropertyPanel`, `ProcessFlowSimSideWidget`, `ProcessFlowJobSetPanel`, `ProcessFlowReportPanel`, `ProcessFlowResultDialog`, `ProcessFlowGanttWidget` |
| 节点模型 | `ProcessFlowNodeProps` |
| 仿真控制 | `ProcessFlowSimController` |
| 建模 | `sim/SimModelBuilder`, `PlantGraph`, `JobSet`, `SimRunConfig` |
| 引擎 | `sim/DesEngine`, `DispatchPolicies`, `IDispatchPolicy`, `IStationExecutor`, `IScheduler` |
| 结果 | `SimStatistics`, `OperationTrace` |

生命周期要点：

1. `initialize`：创建页面与侧栏控件、注册菜单与文档 save/load 钩子  
2. `enterProcessFlow`：中央页 + `enterProcessFlowSideUi` 左右栏  
3. `shutdown`：注销菜单与 UI，释放控制器  

---

## 相关文档

| 文档 | 说明 |
|------|------|
| [`docs/工艺流程仿真插件/DESIGN_工艺流程仿真插件.md`](../../../docs/工艺流程仿真插件/DESIGN_工艺流程仿真插件.md) | UI / 模块设计 |
| [`docs/工艺流程仿真插件/DESIGN_DES引擎.md`](../../../docs/工艺流程仿真插件/DESIGN_DES引擎.md) | DES 引擎设计 |
| [`docs/工艺流程仿真插件/ACCEPTANCE_工艺流程仿真插件.md`](../../../docs/工艺流程仿真插件/ACCEPTANCE_工艺流程仿真插件.md) | 分阶段验收 |
| [`docs/工艺流程仿真插件/TODO_工艺流程仿真插件.md`](../../../docs/工艺流程仿真插件/TODO_工艺流程仿真插件.md) | 后续待办 |
| [`CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md) | 插件 SDK |
| [`ARCHITECTURE_SUMMARY.md`](../../../ARCHITECTURE_SUMMARY.md) | 宿主插件架构 |

早期 `CONSENSUS_*.md` 中「不做仿真/持久化」等表述已被后续实现覆盖，**以本文与源码为准**。
