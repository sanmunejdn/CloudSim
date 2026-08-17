# ACCEPTANCE — 开发文档全面整理（第二轮）

| 项 | 状态 |
|----|------|
| 已完成专题迁入 `_archive`（robot-kinematics / 网页信号 / 网页设备页） | 通过 |
| 活跃区总索引去掉过期「常读」链接 | 通过 |
| `_archive/INDEX.md` 登记新迁入项 | 通过 |
| `src` DEVELOPER_GUIDE / README 相对路径与 `_archive` 前缀修复 | 通过 |
| `src/README.md`、`src/Plugins/README.md`、HelloAi/Labeling 短 README | 通过 |
| `MODULE_DEVELOPER_GUIDES` 补工作区模式插件行 | 通过 |
| 一等公民断链扫描 `broken_links=0` | 通过 |
| 未改 third_party / node_modules | 通过 |
| 未物理销毁 `_archive` 历史稿（迁入即「下架过时活跃态」） | 通过 |

## 抽查命令

```bash
python docs/开发文档整理/_scan_links_active.py
```
