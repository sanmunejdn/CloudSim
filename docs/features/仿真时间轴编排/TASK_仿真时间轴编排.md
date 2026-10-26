# TASK — 仿真时间轴编排

## 依赖图

```mermaid
flowchart LR
  T0[T0 文档与Bug单] --> T1[T1 Model+JSON]
  T1 --> T2[T2 Timeline UI壳]
  T1 --> T3[T3 Scheduler]
  T2 --> T4[T4 拖拽编辑]
  T3 --> T5[T5 PlaybackTape]
  T4 --> T5
  T5 --> T6[T6 Material关键帧]
  T5 --> T7[T7 工程IO+互斥]
  T6 --> T8[T8 验收与双配置编译]
  T7 --> T8
```

## 原子任务

### T0 — 文档冻结 + BugTracker ✅ #70
### T1 — TimelineStudyTypes + JSON ✅
### T2 — SimulationTimelinePageWidget + Dock Timeline 模式 ✅
### T3 — TimelineScheduler ✅
### T4 — 多轨拖拽编辑 ✅
### T5 — PlaybackTape + scrub ✅
### T6 — Material 关键帧 ✅
### T7 — 工程 IO + 运行互斥 ✅
### T8 — ACCEPTANCE + 编译 ✅

## 实现约束（全局）

- 风格对齐 RobotWidget；注释纯中文、只写 Why
- 不改 ProcessFlow 工作区
- 不扩大到 Appearance 物理
