# ai_config.json 配置说明

配置文件路径：**与 `CloudSim.exe` 同目录** 的 `ai_config.json`。  
首次构建若不存在，会从 `src/App/CloudSim/ai_config.defaults.json` 复制一份。

也可在 AI 助手侧栏点击 **设置** 修改远程 LLM 部分（写入同一文件）。

---

## 1. 顶层字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `hardware_profile` | string | 硬件档位提示。默认 `vram_8gb`：建议单模型常驻、8GB 显存用 3B 级模型。 |
| `parser_priority` | string[] | 全局默认解析顺序：`rules` → `local` → `remote`。 |
| `remote_llm` | object | 可选云端 OpenAI 兼容 API。 |
| `router` | object | 领域路由（默认 `explicit_ui`：由 UI 下拉选择领域）。 |
| `domains` | array | **分域专模**列表（核心）。 |

### parser_priority 取值

| 值 | 含义 |
|----|------|
| `rules` | 规则解析（`AiIntentParser`，零模型，适合「生成长方体…」） |
| `local` | 本地 OpenAI 兼容端点（默认 Ollama `http://127.0.0.1:11434/v1`） |
| `remote` | `remote_llm` 云端 API |

按数组顺序依次尝试，**某一档成功即停止**。

---

## 2. remote_llm（可选云端）

```json
"remote_llm": {
  "enabled": false,
  "base_url": "https://api.openai.com/v1",
  "api_key": "",
  "api_key_env": "OPENAI_API_KEY",
  "model": "gpt-4o-mini",
  "timeout_ms": 60000,
  "temperature": 0.1
}
```

| 字段 | 说明 |
|------|------|
| `enabled` | `true` 时允许走 `remote` 解析档 |
| `api_key` | 明文密钥（勿提交 git） |
| `api_key_env` | `api_key` 为空时从环境变量读取 |

---

## 3. domains[]（分域专模）

每个元素对应一个 **AiDomain**（业务场景 + 独立模型槽位）。

```json
{
  "id": "mesh.create",
  "enabled": true,
  "base_url": "http://127.0.0.1:11434/v1",
  "model": "qwen2.5:3b",
  "multimodal": false,
  "parser_priority": ["rules", "local"],
  "unload_other_models_before_infer": false
}
```

| 字段 | 说明 |
|------|------|
| `id` | 稳定领域 ID，发布后勿改。内置：`mesh.create`、`geometry.recognize` 等。 |
| `enabled` | 是否参与解析 |
| `base_url` | 该域本地推理 API 根路径（须含 `/v1`） |
| `model` | Ollama 模型名或 Modelfile 导出名 |
| `multimodal` | `true` 时请求可带截图（vision） |
| `parser_priority` | 覆盖全局顺序；识别域常用 `["local"]` |
| `unload_other_models_before_infer` | `true` 时推理前释放其它已加载模型（8GB 显存建议对 VL 域开启） |

### 出厂默认（8GB 显存）

| id | model | 说明 |
|----|-------|------|
| `mesh.create` | `qwen2.5:3b` | 文本 → 创建基本体 JSON |
| `geometry.recognize` | `qwen2.5vl:3b` | 多模态几何识别 |

---

## 4. 本地 Ollama 部署

推荐将程序与权重放在 **`D:\Project\VSprogram\Ollama`**：

- 环境变量：`OLLAMA_MODELS=D:\Project\VSprogram\Ollama\models`
- API：`http://127.0.0.1:11434/v1`

```powershell
cd D:\Project\VSprogram\Ollama
.\status-ollama.ps1          # 检查服务与模型
.\pull-cloudsim-models.ps1   # 拉取出厂模型（若未拉过）
```

`ai_config.json` 中 `base_url` 须与 Ollama 监听地址一致。本地 Ollama 无需 `api_key`（程序会自动使用占位 Bearer）。

---

## 5. 兼容旧版 ai_config

仍支持顶层扁平字段（映射到 `remote_llm`）：

- `enabled`、`base_url`、`model`、`api_key`、`api_key_env`
- `rule_parser_first`：`true` 时把 `mesh.create` 的 `parser_priority` 设为 `["rules","local","remote"]`

---

## 6. 常见问题

| 现象 | 处理 |
|------|------|
| 输入后无反应 | 确认已用新 `Widget.dll`；插件加载后 `IAiAssistantHost` 已绑定；看运行输出 |
| 创建类语句无模型 | 确认 `parser_priority` 含 `rules`；或 Ollama 已启动 |
| 闲聊崩溃/无响应 | 已修复：勿在后台线程直接更新 UI；闲聊走提示文案 |
| 11434 端口占用 | Ollama 已在运行，无需再 `ollama serve` |
| 换模型不生效 | 修改 `domains[].model` 后重启 CloudSim；Ollama 侧 `ollama list` 确认名称 |

---

## 7. 相关文档

- 训练与导出模型：[`README.md`](README.md)
- SDK 与接模：[`../../src/Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md`](../../src/Plugins/CloudSimAiSDK/DEVELOPER_GUIDE.md)
