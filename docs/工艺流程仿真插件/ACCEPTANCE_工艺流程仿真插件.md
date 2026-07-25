# ACCEPTANCE — 仿真五期优化

| 波次 | 项 | 状态 |
|------|----|------|
| W1 | 甘特图（机器×时间条，悬停/滚轮缩放） | 完成 |
| W2 | LPT/EDD/CR + 对比全部策略 | 完成 |
| W3 | JobSet 编辑面板 + processFlow.jobSet | 完成 |
| W4 | MTBF/MTTR、assembly 汇合、batch | 完成 |
| W5 | 画布忙闲色 + Trace 回放滑条/token | 完成 |

## 手工抽检

1. 运行仿真 → 甘特 Tab 有条带，与 Trace 时间一致  
2. 「对比全部策略」→ 对比表有 FIFO/SPT/LPT/EDD/CR  
3. JobSet 加模板/从路径生成 → 保存工程重开仍在  
4. 工位设 mtbf/mttr 或 batch/assembly 后仿真可跑通  
5. 回放按钮 + 滑条 → 画布节点红框忙闲、蓝点 token  
