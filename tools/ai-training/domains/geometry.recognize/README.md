# geometry.recognize 专模

**场景：** 活动 3D 视口截图 + 文本 → 基本体识别 JSON → 用户确认后 `createPrimitiveMesh`。

| 项目 | 值 |
|------|-----|
| 基座模型 | `qwen2.5vl:3b`（多模态） |
| `multimodal` | `true` |
| `unload_other_models_before_infer` | 建议 `true`（8GB 显存） |
| 训练集 | `dataset.jsonl` + `images/*.png`（`gen_geometry_recognize_dataset.py` 生成） |

## 运行时验收

1. 场景中有 box/cylinder/cone/sphere → AI 选「几何识别」→ 发送「识别这个形状」→ 助手显示类型与尺寸，**场景不变化**。
2. 点击「创建基本体」→ 按识别尺寸创建 mesh。
3. 无视口 / 空文档 → 明确错误提示。
4. `build_dataset.py geometry.recognize` 通过；Ollama VL 模型可收到带图请求。

训练需 VL 基座与带图样本；见 [`../../README.md`](../../README.md) §4.3。
