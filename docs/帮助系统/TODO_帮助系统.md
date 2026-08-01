# TODO — 帮助系统

## 已完成（本期）

- [x] 多页中/英用户手册（18 章）
- [x] 帮助窗口：左侧正文 + 右侧章节列表
- [x] 功能示意图 PNG（`help/images/`，各章配图）
- [x] 生成脚本：`_generate_manual.py`、`_generate_images.py`

## 后续可增强

- [ ] 用真实软件截图替换示意图
- [ ] 按 FEATURES/ROADMAP 增量同步文案

## 维护指引

1. 改文案：编辑 `_generate_manual.py` → `python CloudSim/help/_generate_manual.py`
2. 改图：编辑 `_generate_images.py` → `python CloudSim/help/_generate_images.py`
3. 构建或拷贝 `CloudSim/help` 到 `bin\x64(d)\help\`
