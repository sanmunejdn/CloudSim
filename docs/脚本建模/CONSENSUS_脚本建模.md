# CONSENSUS — 脚本建模

## 需求与验收

1. Ribbon：导出当前 Body history；导入替换；导入新建；运行 compose JSON
2. 根结构判别：`domain==feature.compose` → compose；含 `features` 或 `parametricHistory` → history
3. Undo：替换 history 走 `BodyHistoryCmd`
4. `cloudsim_geom`：`export_history` / `import_history` / `run_compose` / `list_bodies` + 模式内控制台
5. Debug|x64 + Release|x64：GeometricModelingPlugin（及改动的 Host 若有）

## 技术方案

- 插件直调现有 `IPluginGeometryHost` + `IAiAssistantHost`（一期不 bump）
- 新建 Body：先 `extrudeSketchProfileToBrep`（空 target）建最小 Pad，再 `setParametricBodyHistoryJson` 覆盖
- Python：插件内 pybind，动态注入 `sys.modules['cloudsim_geom']`

## 约束

- 不另造特征语义；Shell/Draft 等依赖 face 索引的限制照旧
- 示例见 `examples/`
