# TASK：外部轴联动求解

## 任务依赖

```mermaid
flowchart TD
  T1[T1 模型与 JSON] --> T2[T2 文档 IO]
  T1 --> T3[T3 UI Tab]
  T2 --> T3
  T3 --> T4[T4 Controller 同步与门禁注入]
  T5[T5 Kinematics 解析 J] --> T7[T7 联立微调]
  T6[T6 TeachIk 掩码限幅] --> T7
  T4 --> T8[T8 ExternalAxisSearch]
  T6 --> T8
  T7 --> T8
  T8 --> T9[T9 验收文档]
  T8 --> T10[T10 Run 播放插帧]
  T9 --> T10
```

| ID | 内容 | 验收 |
|----|------|------|
| T1 | `RobotExternalAxes.*` + 实例字段 | 编译、JSON 往返 |
| T2 | ProjectIo / Restore | 存盘重开保留 |
| T3 | SettingsWidget + Dock | Tab 可见可编辑 |
| T4 | Controller sync + Engine 注入 | 选机同步；管道带配置 |
| T5 | 闭式 MDH + 解析 J + prismatic | 自检误差 |
| T6 | TeachIk 外轴 DOF / 限幅 | mm 步长 |
| T7 | 联立 refine 开关 | Search 后可选 |
| T8 | 真搜索 + 去假 rail | 门禁行为 |
| T9 | ACCEPTANCE/FINAL/TODO + guides | 文档齐全 |
| T10 | Run 播放：外轴插值 + 保留轨迹 + 地轨时长 | 点云/多点不瞬移；编译通过 |
