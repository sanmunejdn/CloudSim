# ACCEPTANCE — design.parts

## 交付

| 项 | 路径 |
|----|------|
| 零件资产 | `CloudSim/tools/design-calc/parts/` |
| Python instantiate | `design_parts/instantiate` |
| CLI | `parts_cli.py` |
| C++ Domain | `DesignPartsCatalog` / `DesignPartsDomainHandler` |
| Domain id | `design.parts`（`AiDomainIds::designParts()`） |
| 配置 | `ai_config.defaults.json` 已加 domains 项 |

## 测试

```text
cd CloudSim/tools/design-calc
python -m unittest tests.test_parts -v   # 5 passed
python parts_cli.py parse "创建六角螺栓 M8×30"
```

## 使用

- AI 面板选「标准件」或自动路由命中螺栓/螺母/垫圈/销/齿轮毛坯话术
- 环境变量 `CLOUDSIM_DESIGN_PARTS` 可覆盖库根目录
- 构建 CloudSim 后：`$(OutDir)resource/design-parts`

## 已知限制

- 毛坯无螺纹/齿廓；二次编辑靠改 params 再 instantiate 或手改特征
- **六角螺栓**：一体 `revolveSketchProfileToBrep` 阶梯毛坯（圆柱头近似，非六角对边）；避免两步 Pad 只留下头部
- `design.parts` 规则失败时**不再回落 LLM**（防止只生成六角棱柱）
- UI「确认规格」面板：当前走解析→ActionPlan 执行链；精细规格下拉可后续接 `beginDomainConfirmAsync`
- `design.calc` Domain 尚未嵌入 Host（公式仍 CLI）
