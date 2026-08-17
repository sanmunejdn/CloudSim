# design.calc 可迁移资产目录

**源表：** `非标设计最强自动计算.xls`  
**版本标注（表内）：** V16.SP2（2016.05.03 更新）  
**扫描日期：** 2026-08-17  
**工作表数：** 75（含空表 `分割器选型基础知识`、`Sheet1`）  
**扫描产物：** `_sheet_scan.json`（全表预览）、`_focus_labels.json`（重点表标签）

---

## 1. 定位与用法

本目录回答：**Excel 里哪些可抽成 CloudSim `design.calc` 领域资产**。

| 资产类型 | 含义 | Agent 用法 |
|----------|------|------------|
| `calc` | 有明确输入→公式→输出 | `design.calc` API：填参得结果 |
| `lookup` | 型号/材料/标准参数表 | 选型检索，再进 calc |
| `geometry_dim` | 齿轮/蜗杆等几何尺寸链 | 输出喂给 `feature.compose` / 零件模板 |
| `ref` | 常识、单位、系数 | Skill 附录 / 常量库，不单独成 API |
| `defer` | 宏依赖强、繁体专项、空表、过时表 | 暂不迁，或仅人工参考 |

**原则：** 不把 `.xls` 原文件当运行时；抽成可单测的 JSON schema + 计算函数（或查表）。几何仍走 OCC / 特征史。

---

## 2. 迁移优先级总览

| 优先级 | 目标场景 | 建议先迁的 calc_id |
|--------|----------|-------------------|
| **P0** | 电机/减速链选型闭环（设计减速器前置计算） | `motor.power`、`reducer.rated_power`、`motor.y_series_lookup`、`load.torque`、`inertia.shape`、`servo.select` |
| **P0** | 圆柱/锥齿轮几何 → 可接 CAD 参数 | `gear.spur_helical_shift`、`gear.rack`、`gear.bevel_design`、`gear.high_shift_dims` |
| **P1** | 蜗杆、同步带、链传动 | `worm.geometry`、`worm.param_recommend`、`belt.sync_pulley`、`chain.sprocket`、`chain.params` |
| **P1** | 轴系连接件强度 | `key.strength`、`pin.strength`、`interference.fit`、`coupling.gear`、`coupling.universal` |
| **P2** | 气动、丝杠、分度、螺纹紧固标准 | `cylinder.*`、`ballscrew.*`、`indexer.*`、`thread.*`、`bolt.torque` |
| **P3 / defer** | 材料价格、模具钢大全、摇摆资料、空表、超大「单变量求解」表需人工拆公式 | 见 §5 |

---

## 3. 建议 Domain 分包（`design.calc` 子模块）

```text
design.calc/
  motor/          # 功率、转矩、惯量、Y系列、伺服选型
  reducer/        # 减速机公称功率、传动比验算
  gear/           # 圆柱/斜齿/锥齿/齿条/变位/跨棒距
  worm/           # 蜗杆几何、材料、参数搭配
  belt_chain/     # 同步带、三角带、链轮链条
  shaft_joint/    # 键销、过盈、联轴器、花键检验
  actuator/       # 气缸、真空、电磁阀
  motion/         # 丝杠水平/垂直、皮带轮连续/间歇
  fastener/       # 螺纹标准、螺栓扭矩、攻丝底孔
  material/       # 摩擦、弹模、齿轮材料力学性能
  util/           # 单位换算、常量
```

对外 Agent 可只暴露粗粒度工具，例如：

- `calc_motor_select`
- `calc_reducer_power`
- `calc_gear_geometry`
- `lookup_y_motor`

---

## 4. 可迁移条目目录（按源表）

> **难度：** L1 小表直迁 · L2 多分支/查表 · L3 大表+迭代求解/Excel 宏语义需重写  
> **CAD 耦合：** `param` 仅数字 · `geom` 可驱动特征/模板 · `none` 选型/校核

### 4.1 电机与减速（P0）

| calc_id | 源表# | 表名 | 类型 | 典型输入 | 典型输出 | CAD | 难度 | 备注 |
|---------|-------|------|------|----------|----------|-----|------|------|
| `motor.power` | 30 | 电机功率确定程序 | calc | Vw,F,η / Mw,nw / Pd,nd | Pw, Md | none | L1 | 三套小公式，宜先迁 |
| `motor.y_series_lookup` | 58 | 常用Y系列电机型号参数表 | lookup | P 需求、极数/转速偏好 | 型号,P,n,I,η,cosφ | none | L1 | 约 100 行，纯查表 |
| `motor.trivia` | 69 | 电机常识 | ref | — | 文案 | none | L1 | Skill 附录 |
| `servo.select` | 25 | 伺服电机选型自动版 | calc+lookup | M,丝杠P/D、行程、加减速 | 惯量比、转矩、建议伺服 | none | L3 | 82×19，分支多，拆子 API |
| `load.torque` | 61 | 负载转矩计算 | calc | F,μ,i,PB,机构类型 | 摩擦/负载转矩 | none | L2 | 滚珠丝杠等机构分支 |
| `inertia.shape` | 62 | 惯量计算 | calc | 外径/内径/长/密度/偏心 | J0,Jx,Jy | none | L2 | 多形状公式 |
| `inertia.distance` | 72 | 惯性距计算 | calc | （小表） | 惯性距 | none | L1 | 与 62 可合并 |
| `reducer.rated_power` | 9 | 减速机公称功率 | calc | KA,KS,P,N,n | P2m,i,选型结论 | none | L1 | **减速器选型核心**；结论依赖厂表 |

### 4.2 齿轮几何与设计（P0–P1）

| calc_id | 源表# | 表名 | 类型 | 典型输入 | 典型输出 | CAD | 难度 | 备注 |
|---------|-------|------|------|----------|----------|-----|------|------|
| `gear.spur_helical_shift` | 31 | 变位圆柱 | geometry_dim | Z1,Z2,m/mf,β,A | A0,y,α',ξ,齿顶/根圆等 | geom | L2 | 直/斜/人字三列；接齿轮模板 |
| `gear.rack` | 32 | 齿轮齿条 | geometry_dim | Z,M,β,n,X | d,ha,hf,da,df,V | geom | L1 | |
| `gear.rack_design` | 50 | 齿轮齿条传动设计计算 | geometry_dim | mn,Z,β,b,精度… | 全套几何+验算 | geom | L3 | 320 行，含「单变量求解」→须重写求解器 |
| `gear.bevel_design` | 59 | 锥齿轮传动设计计算 | geometry_dim | Z1,Z2,m,齿型,旋向… | 锥齿几何/刀盘可行性 | geom | L3 | 310 行，GLEASON；同需拆公式 |
| `gear.high_shift_dims` | 56 | 高变位齿轮尺寸计算 | geometry_dim | Z,m,X,α,β,k | Wk,d,da,df,h… | geom | L2 | 公法线/跨棒相关 |
| `gear.helical_span` | 53 | 高度变位斜齿轮跨棒(球)距 | geometry_dim | 变位斜齿参数 | 跨棒/跨球距 | none | L2 | 检验尺寸，非造型必须 |
| `gear.materials` | 33/51 | 齿轮常用材料及其力学性能 | lookup | 材料牌号 | σ、硬度等 | none | L1 | 33 与 51 重复，迁一份 |
| `gear.cam_index_power` | 3 | 齒輪分割計算 | calc | N,t1,t2,n,齿轮/夹具质量… | Tt,Te,Tc,P | none | L2 | 繁体；凸轮分割场景，非通用齿轮 |

### 4.3 蜗杆（P1）

| calc_id | 源表# | 表名 | 类型 | CAD | 难度 | 备注 |
|---------|-------|------|------|-----|------|------|
| `worm.geometry` | 34 | 圆柱蜗杆传动基本几何尺寸计算公式 | geometry_dim | geom | L2 | |
| `worm.materials` | 35 | 蜗杆常用材料 | lookup | none | L1 | |
| `worm.param_recommend` | 36 | 圆柱蜗杆传动主要参数搭配推荐值 | lookup | none | L1 | 选型搭配表 |
| `worm.wheel` | 52 | 蜗轮传动 | geometry_dim | geom | L2 | |

### 4.4 带 / 链传动（P1）

| calc_id | 源表# | 表名 | 类型 | CAD | 难度 | 备注 |
|---------|-------|------|------|-----|------|------|
| `belt.sync_pulley` | 15 | 同步带轮传动设计计算 | calc | geom/none | L2 | |
| `belt.v_params` | 21 | 三角皮带参数表 | lookup | none | L1 | |
| `belt.v_length` | 37 | 三角皮带长度计算 | calc | none | L1 | |
| `belt.pulley_intermittent` | 65 | 皮带轮间歇运动 | calc | none | L2 | 与伺服动作相关 |
| `belt.pulley_continuous` | 66 | 皮带轮连续运动 | calc | none | L2 | |
| `chain.sprocket` | 10 | 链轮计算 | calc | geom | L2 | |
| `chain.params` | 46 | 链轮参数计算 | geometry_dim | geom | L2 | |
| `chain.link` | 11 | 链条计算 | calc | none | L2 | |
| `conveyor.belt` | 2 | 輸送帶計算 | calc | none | L2 | 繁体；产线输送，可选 |

### 4.5 轴系连接与配合（P1）

| calc_id | 源表# | 表名 | 类型 | CAD | 难度 | 备注 |
|---------|-------|------|------|-----|------|------|
| `key.strength` | 14 | 键的强度计算 | calc | none | L1 | |
| `pin.strength` | 13 | 销的强度计算 | calc | none | L1 | |
| `weld_vs_key` | 12 | 焊缝及键连接受力计算比较 | calc | none | L2 | |
| `interference.fit` | 8 | 过盈计算 | calc | none | L2 | |
| `coupling.gear` | 6 | 齿式联轴器计算 | calc | none | L1 | |
| `coupling.universal` | 7 | 万向联轴器计算 | calc | none | L1 | |
| `clutch.overrunning` | 57 | 超越离合器设计计算表 | calc | none | L2 | |
| `spline.external_span` | 54 | 外花键跨棒距 | geometry_dim | none | L2 | 检验 |
| `spline.internal_span` | 55 | 内花键棒间距 | geometry_dim | none | L2 | 检验 |
| `ratchet.dims` | 4 | 棘轮计算 | geometry_dim | geom | L2 | 料带 PITCH 场景 |

### 4.6 气动 / 真空（P2）

| calc_id | 源表# | 表名 | 类型 | 难度 |
|---------|-------|------|------|------|
| `cylinder.bore_select` | 24 | 气缸内径选型 | lookup/calc | L1 |
| `cylinder.thrust` | 23 | 气缸推力计算 | calc | L1 |
| `cylinder.force_table` | 22 | 气缸理论出力表 | lookup | L1 |
| `cylinder.system_guide` | 20 | 气缸与系统选型指南 | ref | L1 |
| `pneumatic.air_consume` | 19 | 耗气量计算及电磁阀选择 | calc | L2 |
| `vacuum.component` | 18 | 真空元件的选定 | lookup | L2 |

### 4.7 丝杠与直线运动（P2）

| calc_id | 源表# | 表名 | 类型 | 难度 | 备注 |
|---------|-------|------|------|------|------|
| `ballscrew.horizontal` | 63 | 丝杠水平运动 | calc | L2 | 与 `servo.select` 可串联 |
| `ballscrew.vertical` | 64 | 丝杠垂直运动 | calc | L2 | |
| `column.stability` | 47 | 立柱计算 | calc | L2 | |
| `stability.factor` | 48 | 稳定性系数 | lookup | L1 | |
| `press.fit_force` | 27 | 压入力计算 | calc | L1 | |

### 4.8 分度 / 凸轮分割器（P2）

| calc_id | 源表# | 表名 | 类型 | 难度 | 备注 |
|---------|-------|------|------|------|------|
| `indexer.select_formula` | 16 | 分度盘选型计算公式 | calc | L2 | |
| `indexer.disk` | 67 | 分度盘 | calc | L2 | |
| `indexer.calc` | 73 | 分割器计算 | calc | L2 | |
| `disk.round` | 70 | 圆盘 | calc | L1 | |
| `indexer.basics` | 71 | 分割器选型基础知识 | — | — | **空表，跳过** |

### 4.9 螺纹与紧固（P2）

| calc_id | 源表# | 表名 | 类型 | 难度 |
|---------|-------|------|------|------|
| `thread.unc` | 38 | 美制螺纹 | lookup | L1 |
| `thread.coarse` | 39 | 粗螺纹 | lookup | L1 |
| `thread.fine` | 40 | 细螺纹 | lookup | L1 |
| `thread.tap_drill` | 41 | 迫牙丝攻钻孔径 | lookup | L1 |
| `thread.extra_fine` | 42 | 美制特细螺纹及英制电器螺纹 | lookup | L1 |
| `thread.pipe` | 43 | 管螺纹 | lookup | L1 |
| `thread.sewing` | 44 | 螺纹及针车用螺纹 | lookup | L1 |
| `bolt.torque` | 45 | 螺栓扭矩标准 | lookup | L1 |

### 4.10 材料与杂项（P2–P3）

| calc_id | 源表# | 表名 | 类型 | 难度 | 备注 |
|---------|-------|------|------|------|------|
| `material.friction` | 29 | 材料摩擦系数 | lookup | L1 | |
| `material.elastic` | 28 | 弹性模量、泊松系数 | lookup | L1 | |
| `material.mold_steel` | 49 | 模 具 钢 | lookup | L2 | 宽表，非传动优先 |
| `material.price` | 26 | 材料价格计算表 | calc | L2 | 商务向，可延后 |
| `util.unit_convert` | 17 | 单位换算 | calc | L1 | |
| `spring.calc` | 5 | 弹簧计算 | calc | L2 | |
| `overview` | 60 | 概述 | ref | L1 | |
| `misc` | 68 | 其他 | ref | L1 | |

### 4.11 暂缓 / 需人工二次拆解（defer）

| 源表# | 表名 | 原因 |
|-------|------|------|
| 1 | 搖擺資料 | 繁体资料页，非标准 calc API |
| 50 / 59 | 齿轮齿条设计 / 锥齿轮设计 | 依赖 Excel「单变量求解」与大量黄色判断格；须按手册重写算法后再迁 |
| 71 / 74 | 空表 | 无内容 |
| （目录提及）轴承大全 | 查询深沟球轴承尺寸 | **本工作簿未见独立可用 sheet**；目录有名无表，勿承诺 |

---

## 5. 推荐落地顺序（可验收）

### Wave A — 最小闭环（支撑「算减速链」）

1. `util.unit_convert`  
2. `motor.power` + `motor.y_series_lookup`  
3. `reducer.rated_power`  
4. `load.torque` + `inertia.shape`（子集：圆柱/圆盘）  

**验收：** 给定 P、n_in、n_out、KA/KS → 输出 i、计算功率、候选 Y 电机型号。

### Wave B — 齿轮参数 → CAD

1. `gear.spur_helical_shift`  
2. `gear.rack`  
3. `gear.materials`  
4. 输出 schema 对接 `feature.compose` / 齿轮零件模板（模数、齿数、变位、齿宽）  

**验收：** 给定 Z1,Z2,m,β,A → 输出 da/df/d 等，并能创建或填模板（不一定含齿廓精确建模）。

### Wave C — 传动扩展

蜗杆、同步带、链、键销强度、过盈。

### Wave D — 大表重写

`gear.rack_design`、`gear.bevel_design`、`servo.select`：对照《齿轮手册》/原表说明拆算法，**禁止**依赖 Excel 迭代宏。

---

## 6. 建议资产文件形态

```text
tools/design-calc/                 # 或 CloudSim/tools/design-calc/
  catalog.json                     # 本目录机器可读摘要
  schemas/<calc_id>.schema.json    # 输入输出 JSON Schema
  impl/<calc_id>.py                # 或 C++ 等价实现 + 单测
  tables/<lookup_id>.csv           # 从 xls 导出的查表
  fixtures/<calc_id>/*.json        # 用原表样例做金标
```

单条 `catalog.json` 字段建议：

```json
{
  "id": "reducer.rated_power",
  "source_sheet": 9,
  "source_name": "减速机公称功率",
  "kind": "calc",
  "priority": "P0",
  "inputs": ["KA", "KS", "P_kW", "N_rpm", "n_rpm"],
  "outputs": ["P2m_kW", "i", "note"],
  "cad_coupling": "none",
  "difficulty": "L1",
  "status": "planned"
}
```

---

## 7. 风险与合规

| 项 | 说明 |
|----|------|
| 版权 | 表为第三方「非标设计」汇编；入库前确认可商用/可改写公式 |
| 标准年代 | 多处引用 GB 10095/10096-88、GB11365-89 等，需标「公式年代」与适用范围 |
| 单位混用 | 同时出现 kg·m、HP、kW、cm；迁移时统一 SI，保留单位字段 |
| 繁简混排 | 部分 sheet 繁体（輸送帶、齒輪分割）；API 对外统一简体 id |
| 与 CAD | 几何尺寸可驱动模板；**齿廓精确造型**需另开齿轮特征/库，不算本表已交付 |

---

## 8. 对「设计减速器」Agent 的直接映射

| 用户意图片段 | 优先资产 |
|--------------|----------|
| 功率/转速/传动比 | `motor.power`、`reducer.rated_power`、`motor.y_series_lookup` |
| 惯量与负载转矩 | `inertia.shape`、`load.torque`、`servo.select`（后续） |
| 齿轮副尺寸 | `gear.spur_helical_shift` → CAD 模板 |
| 蜗轮蜗杆减速 | `worm.*` |
| 键连接校核 | `key.strength` |

**本表不能单独完成：** 行星/斜齿箱装配、精确齿面、箱体造型、干涉——仍需装配 IR + 零件库 + OCC 特征。

---

## 9. 落地状态（已确认）

| 决策 | 状态 |
|------|------|
| 公式写入仓库 | ✅ `tools/design-calc` |
| Wave A/B/C | ✅ 见 ACCEPTANCE |
| 接 feature.compose/模板 | ✅ 毛坯 compose + `templates/` |

运行时 AI Domain 接线见 ACCEPTANCE TODO。
