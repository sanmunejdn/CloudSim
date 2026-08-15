# ALIGNMENT — 旋转副运动中心绑 Frame

## 决策

只做 **自定义坐标系**：旋转中心绑定已有 `FrameBackendData`。

## 行为

- 「旋转中心」下拉：
  - （无）
  - **模型：…** — 组装画布上各连杆几何自身坐标系（原点）
  - **坐标系：…** — 场景已有 Frame
- 存 `motion.motionCenterFrameBackendId`（Frame 或模型 Backend id）
- FK / Apply：将该 Backend 原点变到父连杆局部 → `originMm`
