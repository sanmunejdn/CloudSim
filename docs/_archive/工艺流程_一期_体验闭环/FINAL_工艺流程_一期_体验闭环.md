# FINAL — 工艺流程一期体验闭环

## 已交付
- 工程脏标记：`markActiveDocumentModified`（Host 1.36）+ 退出工艺流程确认
- 缓冲报表：`BufferStat` 标题为「机A→机B」；ReportPanel 缓冲表；CSV 扩列
- 多策略甘特：对比可勾选「对比含甘特」，ResultDialog 策略下拉切换甘特
- AI `patchProcessFlowGraph` + Confirm 大 JSON 摘要
- 仿真后可选打开甘特

## 验收
进入流程改图 → 页签出现 *；对比含甘特可见多策略；AI「工位2节拍改为45」走 patch。
