# mesh.compose 专模（布尔多步编排）

**场景：** 自然语言 → ActionPlan v2（多步 `createPrimitiveMesh` + `booleanMesh`）。

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5:3b` |
| 自训练示例名 | `cloudsim-compose:3b` |
| 输出 | `{"version":2,"steps":[...]}` |
| 数据集 | 本目录 `dataset.jsonl` |

## 布尔 op 与口语关键词

| op | 中文 | 英文 | 典型几何 |
|----|------|------|----------|
| `difference` | 挖、孔、通孔、盲孔、减去 | hole, drill, subtract | box + cylinder（刀具） |
| `union` | 并集、合并、拼合、附加凸台 | union, merge, combine | box + box（常需 `pose_mm` 错位） |
| `intersection` | 交集、求交、重叠部分 | intersection, intersect | box + box 或 box + cylinder |

## 金标示例

### 通孔（difference）

用户：「生成长方体 100×100×100，顶面 D50 通孔」

1. `createPrimitiveMesh` box → `id: body`
2. `createPrimitiveMesh` cylinder R25 H120 → `id: hole_tool`
3. `booleanMesh` `op:difference` `target:$body` `tool:$hole_tool`

### 错位并集（union）

用户：「80 和 60 的两个盒子并在一起，小盒 X 偏 40」

1. box 80³ → `id: a`
2. box 60³，`pose_mm: {x:40,y:0,z:0}` → `id: b`
3. `booleanMesh` `op:union` `target:$a` `tool:$b`

### 错位交集（intersection）

用户：「100 和 80 盒子求交，80 盒 X 偏 50」

1. box 100³ → `id: a`
2. box 80³，`pose_mm: {x:50,y:0,z:0}` → `id: b`
3. `booleanMesh` `op:intersection` `target:$a` `tool:$b`

## 生成与校验

```bash
cd CloudSim/tools/ai-training
python scripts/gen_mesh_compose_dataset.py
python scripts/build_dataset.py mesh.compose
```

目标占比（首版 ~50 条）：difference ~36%，union ~36%，intersection ~24%，create_only 2 条。

## 训练前运行时门禁

重新编译 `Data.dll` 后，布尔自检应包含 union/intersection 错位双盒（见 `MeshBoolean::runSelfTest`）。

## 微调与部署

### LLaMA-Factory 数据集注册（示例）

```json
{
  "cloudsim_mesh_compose": {
    "file_name": "domains/mesh.compose/dataset.jsonl",
    "formatting": "alpaca",
    "columns": {
      "prompt": "instruction",
      "query": "input",
      "response": "output"
    }
  }
}
```

训练命令见 [`../../README.md`](../../README.md) §5；导出后：

```bash
ollama create cloudsim-compose:3b -f Modelfile
```

`ai_config.json`：

```json
{
  "id": "mesh.compose",
  "model": "cloudsim-compose:3b",
  "parser_priority": ["rules", "local"]
}
```

通孔仍可走 **rules**；union/intersection 依赖 **local** 专模。

## CloudSim 验收清单

| 输入 | 期望 op |
|------|---------|
| 两个长方体并集，80 和 60，小盒 X 偏 40 | union |
| union two boxes 80 and 60 offset x 40 | union |
| 100 和 80 盒子求交，80 盒 X 偏 50 | intersection |
| 100 盒挖 D50 通孔 | difference（rules 或 local） |

通过标准：JSON 可解析、场景树 **1 个结果 mesh**、无 Step failed。
