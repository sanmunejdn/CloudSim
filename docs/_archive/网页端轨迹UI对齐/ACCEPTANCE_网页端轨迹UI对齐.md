# ACCEPTANCE — 网页端轨迹 UI 对齐

| 项 | 状态 |
|----|------|
| 特征表 5 列 | 通过（前端） |
| feature-schema API | 通过（Host+Gateway Debug/Release） |
| FaceParamSurface 全参数表单 | 通过（schema 驱动） |
| 400ms 自动再离散 | 通过 |
| 删面/边 chip + 删特征 | 通过 |
| 离散模板 存/载/列/删 | 通过 |
| 拾取状态文案 | 通过 |
| 桌面仿分区布局 | 通过 |
| Debug\|x64 + Release\|x64 | 通过 |
| public-fallback 部署 | 通过 |

## 待用户手测

1. 重启 CloudSimWeb，Ctrl+F5
2. 开始修改 → 拾取面 → 策略选「参数面扫描」→ 改行/列间距，确认 Raw 点数变化
3. 点 chip × 删除面索引后自动再离散
4. 存/载离散模板
