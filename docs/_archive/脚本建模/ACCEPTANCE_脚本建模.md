# ACCEPTANCE — 脚本建模

| 项 | 结果 |
|----|------|
| 6A + examples（history / compose） | 已交付 |
| Ribbon：导出 / 导入替换 / 导入新建 / 运行 Compose | 已交付 |
| `ScriptModelIo` 格式判别 | 已交付 |
| `cloudsim_geom` 四 API + 控制台 | 已交付 |
| 全特征/草图 SCHEMA + templates | 已交付（`SCHEMA.md`、`templates/`） |
| FEATURES / ROADMAP | 已更新 |
| Debug\|x64 + Release\|x64 Plugin | **通过** |

## 手工点验

1. 用示例 `box_pad.history.json`：导入新建 → 出现 40×40×10 方块
2. 导出再导入替换：特征树一致
3. 运行 `plate_holes.compose.json`：板+通孔
4. Python：`import cloudsim_geom as g; print(g.list_bodies())`
