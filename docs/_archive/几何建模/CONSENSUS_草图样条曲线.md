# CONSENSUS — 草图样条曲线

## 需求

草图绘制增加**过点插值样条**：左键依次加点，**右键结束提交**，**Esc 取消**未完成样条。

## 验收标准

1. Ribbon 有「样条」工具，可切换并绘制。
2. ≥2 个过点右键可提交；Esc 丢弃未提交点。
3. Overlay 显示平滑曲线；JSON 持久化后重开可还原。
4. 删除工具可点选删除样条。
5. 非构造样条参与封闭轮廓 / 开放路径导出（采样为折线）。
6. 本期不做：约束、Trim/Mirror、控制点拖拽、GCS BSpline。

## 技术方案

- `SkSpline { id, throughPts[], construction }` + Catmull-Rom 采样。
- `SplineSketchTool`：点存工具内，提交时一次性 `addPoint` + `addSpline`。
- 预览：`ISketchTool::previewPolyline`。
- 导出：采样折线并入现有 Line 链逻辑。

## 约束

仅改 GeometricModelingPlugin；Host ABI 不变。
