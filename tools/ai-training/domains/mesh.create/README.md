# mesh.create 专模

**场景：** 自然语言 → 创建 box / cylinder / cone / sphere；**用户可省略尺寸**，运行时与训练金标均用默认或推断后的完整 `dimensions_mm`。

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5:3b`（Ollama） |
| 自训练示例名 | `cloudsim-mesh:3b` |
| 运行时输出 | `create_mesh` v1 或 ActionPlan v2 |
| 数据集 | 本目录 `dataset.jsonl`（约 80 条，含无尺寸/部分/口语样本） |

训练总流程见 [`../../README.md`](../../README.md)。

## 默认尺寸（与运行时一致）

| primitive | dimensions_mm |
|-----------|---------------|
| box | 100 × 100 × 100 mm |
| cylinder | R50, H100 |
| cone | R50, H100 |
| sphere | R50 |

可在 CloudSim 的 `ai_config.json` 中设置 `mesh_create_defaults` 覆盖（见 `ai_config.defaults.json` 与 [`CONFIGURATION.md`](../../CONFIGURATION.md) §1.1）。

## 校验数据

```bash
cd CloudSim/tools/ai-training
python scripts/build_dataset.py mesh.create
```

## 训练集要删吗？

**不要删** `dataset.jsonl`。微调完成后 Ollama 里的是权重；jsonl 用于复训、增样本和 CI 校验。仅删除本机 LLaMA-Factory 的 `saves/` 等临时目录。详见 [`../../README.md`](../../README.md) §2.1。

## 微调后部署到 Ollama

1. 按 [`../../README.md`](../../README.md) §5 完成 LLaMA-Factory SFT 并导出 GGUF（或 Ollama 可导入格式）。
2. 在 Ollama 模型目录创建 `Modelfile`（示例）：

```dockerfile
FROM ./cloudsim-mesh-q4.gguf
PARAMETER temperature 0.1
SYSTEM You convert user requests into create_mesh JSON. Always include full dimensions_mm in mm. If user omits sizes use box 100x100x100, cylinder R50 H100, cone R50 H100, sphere R50; scale for 大一点/小一点/扁/厚.
```

3. 创建模型：

```powershell
cd D:\Project\VSprogram\Ollama
.\env.ps1   # 若用项目脚本
ollama create cloudsim-mesh:3b -f path\to\Modelfile
ollama list
```

4. 修改 CloudSim `ai_config.json`（与 exe 同目录）：

```json
{
  "domains": [
    {
      "id": "mesh.create",
      "model": "cloudsim-mesh:3b",
      "base_url": "http://127.0.0.1:11434/v1",
      "parser_priority": ["rules", "local"]
    }
  ]
}
```

5. 重启 CloudSim；**无尺寸**句（如「生成长方体」）仍优先走 **rules** 与默认补全；口语句（如「大一点的圆柱」）在 rules 未命中时由 **local** 专模解析。

未微调时也可使用 `qwen2.5:3b`：宿主 `meshSystemPrompt` + `AiMeshDefaults` 已支持缺省补全。
