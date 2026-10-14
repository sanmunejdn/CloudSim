# ALIGNMENT — 资源路径统一

> 阶段：Align（对齐）。未获关键决策前不改代码。

## 1. 原始需求

希望 **开发路径统一**：不要同时记 `CloudSim\resource` 与 `bin\resource`（以及 `bin\x64(d)\resource`）多套位置；编辑与查找有明确唯一入口。

上下文：用户曾问能否把 `CloudSim\resource` 合并进 `bin\resource`。

## 2. 现状（项目理解）

### 2.1 三层路径实际在用

| 层级 | 典型路径 | 角色 |
|------|----------|------|
| 模块源 | `CloudSim\src\...\resource\`（trajectory / feature / ExportPython 等） | 文本资源权威源，PostBuild → OutDir |
| CloudSim 顶层源 | `CloudSim\resource\`（ControllerPython、controllers） | 外置控制器权威源，PostBuild → OutDir；C++ 开发态回退也指向此处 |
| 共享仓 | `bin\resource\`（models、部分 trajectory/ExportPython 副本） | 大模型资产 + 打包兜底；URDF 入库脚本默认写这里 |
| 运行时 | `bin\x64\` / `bin\x64d\` 下 `resource\` | `{exe}/resource`，程序实际加载 |

### 2.2 痛点

1. **同名不同责**：两边都叫 `resource`，一边是源、一边是共享仓/副本。
2. **外置控制器特殊**：只有它放在 `CloudSim\resource`，其它脚本/JSON 跟模块；开发文档与对话框回退路径硬编码该位置。
3. **`bin\resource` 与 OutDir 双份**：models/trajectory 等常需手动或脚本同步到 `bin\x64(d)\resource`。

### 2.3 已对齐的工程约束（不可破）

- 运行时根仍是 **`{applicationDir}/resource`**（即 OutDir 下），不改 `OutDir` 约定。
- Debug|x64 / Release|x64 双配置各自 OutDir：`bin\x64d`、`bin\x64`。
- 打包：`collect-deps.ps1` 优先 OutDir，其次 `bin\resource`。

## 3. 任务边界（草案）

| 纳入 | 不纳入（除非另批） |
|------|-------------------|
| 明确「开发编辑权威路径」与「运行时路径」两层约定 | 把整棵 models 迁出 `bin`（体积/入库脚本成本高） |
| 消掉或归并 `CloudSim\resource` 这一「第三入口」 | 改运行时相对路径布局（仍是 `resource/models` 等） |
| 更新 vcxproj 拷贝、C++ 回退、相关 README/验收说明 | 全仓历史文档大扫除 |

## 4. 可选方案（待决策）

### 方案 A — 权威源跟模块；`bin\resource` 只承载大资产（推荐）

- 将 `CloudSim\resource\{Python\ControllerPython,controllers}` **迁到**  
  `CloudSim\src\UI\RobotWidget\resource\`（与 ExportPython 同级）。
- 删除 `CloudSim\resource`；开发改 RobotWidget 下资源；构建拷到 OutDir。
- `bin\resource` **仅保留** `models`（及打包兜底用副本策略不变）。
- 开发口诀：  
  - **改脚本/JSON → 模块旁 `src/.../resource`**  
  - **改模型 → `bin\resource\models`**  
  - **跑程序 → `bin\x64(d)\resource`**

优点：与现有 ExportPython/trajectory 模式一致；源码不进 `bin`。  
缺点：仍有「文本源 vs 模型仓」两处（但职责清晰）。

### 方案 B — 开发一律改 `bin\resource`（用户上次合并方向）

- 把 `CloudSim\resource` 内容并入 `bin\resource`；vcxproj/回退改为从 `bin\resource` 拷贝/读取。
- 开发口诀：开发只看 `bin\resource`；运行仍看 OutDir（构建同步）。

优点：顶层只记一个 `bin\resource`。  
缺点：源码与大资产混在 `bin`；git/协作边界模糊；与其它模块「源在 src」不一致。

### 方案 C — 扩大 `CloudSim\resource` 为唯一文本源仓

- 把 ExportPython、trajectory、feature 等也迁到 `CloudSim\resource\`；模块下不再放 resource。
- `bin\resource` 仍只放 models。

优点：CloudSim 下单一文本源根。  
缺点：改动面大（多工程 PostBuild、文档），超出「消掉双 resource」的最小成本。

## 5. 疑问澄清（需你拍板）

**Q1（关键）选哪套统一策略？**

- **A**：跟模块源 + `bin\resource` 仅 models（推荐）
- **B**：开发权威改为 `bin\resource`
- **C**：扩大 `CloudSim\resource` 为文本总仓

**Q2**：本波是否包含「OutDir 构建时从 `bin\resource\models` 自动同步到 `bin\x64(d)\resource\models`」？  
（现状常靠人工/入库脚本同步；统一开发体验时很有用，但会拉长 PostBuild。）

**Q3**：`bin\resource` 里已有的 `Python\ExportPython`、`trajectory` 副本——统一后是删除、改由构建生成，还是保留作打包兜底？

## 6. 已确认决策（2026-10-14）

- **Q1 → A**：模块源 + `bin/resource` 仅 models
- **Q2 → 是**：构建自动同步 `bin/resource/models` → OutDir
- **Q3 → 删掉**：`bin/resource` 下 ExportPython/trajectory 副本删除，改由构建生成

权威协议见 [CONSENSUS_资源路径统一.md](CONSENSUS_资源路径统一.md)。
