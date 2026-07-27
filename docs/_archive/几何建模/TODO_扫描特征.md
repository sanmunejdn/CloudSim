# TODO — 实体扫描特征

## 已完成（完善 + 续优化）

- [x] 中文 i18n（Unicode 转义）
- [x] 预览失败 logWarn + 提交前预检
- [x] 路径真弧段 MakePipe
- [x] Parametric 自测含 Sweep/SweepCut + pathSketchRefId
- [x] Host 1.31.1
- [x] 改草图后下游 Sweep 重烤 `pathSegments` / profile
- [x] 侧栏扫描状态行显示失败原因
- [x] 轮廓/路径下拉启发式（闭合优先轮廓、开放优先路径）

## 待办（手测 / 后续）

- [ ] GUI 手测 ACCEPTANCE #1–#4（见 `QA_扫描特征_失败模式.md`）
- [ ] 视口点选轮廓/路径（仍为侧栏下拉）
- [ ] 引导线 / 扭转 / 薄壁 / 模型边路径（明确不做）

## 缺配置

无额外配置；确认运行宿主 ≥ 1.31.1，插件 `minHostVersion` 已对齐。
