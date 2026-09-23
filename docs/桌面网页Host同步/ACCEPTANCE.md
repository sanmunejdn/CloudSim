# ACCEPTANCE — 桌面网页 Host 同步落地

| 项 | 状态 |
|----|------|
| `scripts/check_host_headless_sources.py`（检查 / `--fix` / `--list-allow`） | 完成 |
| 当前共享源差集为 0 | 完成（含补齐 9 个 `inc/headless/*Bridge.h`） |
| `PluginPropertyBindingRegistry` 已在 Headless | 完成（前序 LNK 修复） |
| filters sync `CloudSimHostHeadless` | 完成 |
| 文档：`docs/桌面网页Host同步/README.md` + SOURCE / MODULE / Host GUIDE / README 索引 | 完成 |
| Cursor 规则 `host-headless-sync.mdc` | 完成 |
| 废弃 `_gen_headless_vcxproj.py` 全量覆盖 | 完成 |
| Debug\|x64 + Release\|x64 编过 `CloudSimHostHeadless` | 完成 |

未纳入本次范围（产品能力债，非工程同步）：PLC/相机 stub、几何算子写回加厚、PlaneGCS 视口等——见归档 TODO 与能力对等文档。
