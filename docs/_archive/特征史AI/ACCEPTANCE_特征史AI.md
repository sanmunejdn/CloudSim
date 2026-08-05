# ACCEPTANCE — feature.compose + Host 规范收口

| 项 | 状态 |
|----|------|
| CircularPattern 虚表尾部追加 | 完成 |
| feature.compose LLM 解析分支 | 完成 |
| ActionPlan 执行前 validatePlanJson | 完成 |
| Pattern `resolvePatternSeed` 统一 tip fuseOnto | 完成 |
| AI Loft 世界坐标轮廓 | 完成 |
| plugin.json minHostVersion = 1.47.0 | 完成 |
| ABI 1.47.0 `0x00012F00` | 完成 |
| Debug\|x64 编译 | 完成（SDK→CloudSimHost→GeometricModelingPlugin） |

## 手测要点

1. 旧插件勿链新 Host：虚表插错会导致 crash；须同编 1.47+  
2. AI feature.compose 远程 LLM 输出经校验后再执行  
3. Pad→Fillet→Pattern(source=Pad) 预览/提交均保留 Fillet  
4. AI loft 两轮廓 Z 不同时实体非共面 degenerate
