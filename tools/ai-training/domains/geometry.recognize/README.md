# geometry.recognize 专模

**场景：** 截图 + 文本 → 基本体识别 JSON → 可选转 `createPrimitiveMesh`。

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5vl:3b`（多模态） |
| `multimodal` | `true` |
| `unload_other_models_before_infer` | 建议 `true`（8GB 显存） |

训练需 VL 基座与带图样本；见 [`../../README.md`](../../README.md)。
