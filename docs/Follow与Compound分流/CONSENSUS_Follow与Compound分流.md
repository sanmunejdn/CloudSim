# CONSENSUS — Follow 与 Compound 分流

## 需求

- **Follow**：仅跨部件（显式 Follow、设备挂法兰）。
- **Compound**：同部件 Data 子树刚体 \(\Delta=W_{new}\cdot W_{old}^{-1}\)。
- Data 父子边 **不** 自动装 hierarchy Follow；旧工程 `hierarchyDriven` 加载/求解时剥离。

## 流水线

```text
主运动 → Follow → compound/applyQ → 受限 Follow（跟 compound target）
       → refresh 挂载设备 → 再受限 Follow（跟连杆子件）→ flush
```

## 验收

见 ACCEPTANCE_Follow与Compound分流.md。
