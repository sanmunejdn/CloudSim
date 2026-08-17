# design-calc（Wave A/B/C）

自《非标设计最强自动计算》改写的可测计算库，并可导出 **`feature.compose`** 毛坯计划。

## 快速使用

```bash
cd CloudSim/tools/design-calc
python -m unittest tests.test_waves -v
python cli.py reducer
python cli.py gear --compose-out templates/out.compose.json
```

在 CloudSim 几何模式：导入/运行 compose JSON，或 `cloudsim_geom.run_compose(...)`。

## 模块

| Wave | API |
|------|-----|
| A | `motor_power` `lookup_y_motor` `reducer_rated_power` `load_torque_ballscrew` `inertia_cylinder` |
| B | `gear_spur_helical_shift` `gear_rack` `gear_high_shift_dims` `lookup_gear_material` |
| C | `worm_geometry` `key_strength` `belt_v_length` `chain_sprocket_pitch_diameter` |
| CAD | `gear_pair_to_feature_compose` `gear_rack_to_feature_compose` `worm_to_feature_compose` |
| Parts | `design_parts.instantiate` / `parts_cli.py`（标准件库，见 `parts/`） |


## 模板

`templates/*_blank.compose.json`：回转/拉伸**毛坯**（`meta.blank_only=true`），非渐开线齿面。

## 文档

`CloudSim/docs/design.calc/`
