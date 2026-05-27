# mesh.create 专模

**场景：** 自然语言 → 创建 box / cylinder / cone / sphere。

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5:3b`（Ollama） |
| 运行时输出 | `create_mesh` v1 或 ActionPlan v2 |
| 数据集 | 本目录 `dataset.jsonl` |

训练步骤见 [`../../README.md`](../../README.md)。出厂配置下简单句由 **rules** 解析，可不训练。
