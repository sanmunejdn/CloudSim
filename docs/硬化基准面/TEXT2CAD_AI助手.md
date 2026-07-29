# Text-to-CAD → CloudSim AI 助手

参考仓库（GitHub）：

| 项目 | 思想 |
|------|------|
| [earthtojake/text-to-cad](https://github.com/earthtojake/text-to-cad) | Agent Skills：自然语言 → 可执行 CAD 规格（尺寸齐全、可导出 STEP） |
| [SadilKhan/Text2CAD](https://github.com/SadilKhan/Text2CAD) | **顺序特征链**（草图→拉伸→切除…）而非一次性网格 |
| [BoYuanVisionary/Pro-CAD](https://github.com/BoYuanVisionary/Pro-CAD) | **先澄清再绘制**：歧义时提问，避免瞎猜尺寸 |
| [Adam-CAD/CADAM](https://github.com/Adam-CAD/CADAM) | 参数化脚本 + 可调尺寸 |

## 已落地

1. Domain **`feature.compose`**：Pad / Pocket / Fillet / LinearPattern → Parametric Host 真特征史  
2. Domain **`mesh.compose`**：网格布尔（通孔/差集等）保留  
3. `askClarify`、路由拆分、规则 Pad（`tryParseFeatureComposeUserText`）  
4. `onParametricBodyHistoryChanged` → 几何建模特征树 sync  

详见 `../特征史AI/`。

## 下一跳

- Pocket/Chamfer/Revolve 进 ActionPlan  
- 可编辑草图 JSON  
- 混合句「拉伸+通孔」统一到 feature 链
