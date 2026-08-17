# CONSENSUS — design.parts 标准件库

## 决策

1. 标准件以 **文件资产** 存在：`part.json` + `model.compose.json`
2. `instantiate(part_id, params) → feature.compose`
3. AI Domain **`design.parts`** 与 `design.calc` 并列（calc 仍主要为离线库；parts 已接 Host）
4. 当前模型保真度为 **blank**；真螺纹/齿廓通过换 `model_ref` 升级

## 首批家族（5）

| part_id | 说明 |
|---------|------|
| `fastener.hex_bolt_iso4017` | 六角螺栓毛坯 |
| `fastener.hex_nut_iso4032` | 六角螺母毛坯 |
| `fastener.plain_washer_iso7089` | 平垫圈毛坯 |
| `pin.cylindrical_iso2338` | 圆柱销毛坯 |
| `gear.spur_blank` | 直齿轮毛坯 |

## 验收

- Python：`tests/test_parts.py` 通过；`parts_cli.py parse "六角螺栓 M8×30"`
- C++：Domain 注册 `design.parts`；规则可解析并 `execute` → feature.compose
- 运行目录存在 `resource/design-parts`（CloudSim 构建拷贝）或 `CLOUDSIM_DESIGN_PARTS`
