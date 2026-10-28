# 特征表跨工件叠加 — 待办

1. **点验（Debug `bin\x64d` / Release `bin\x64`）**：装配体选零件 A 拾取 → 切零件 B 拾取 → 表内两行「工件」列不同 → 离散预览两段轨迹分别贴合 A、B，段间不连线。
2. **#92**：SelfTestRunner 注入 `FeatureDiscretizerConfigRegistry` 后重跑 `--preset=GeometryAlgorithm`，确认 `featureListEntryWorkpieceJson` 通过。
3. **#91**：`check_host_headless_sources.py` 报 Viewport 3 个 osg 源文件，与本任务无关。
4. **#90**：CloudSimPluginHost moc 找不到 `inc\PluginHostContext.h`，既有问题。
5. 若需要「非首工件移动后轨迹跟随」，需把 RawTrajectory 改为按段记录工件，属新需求。
