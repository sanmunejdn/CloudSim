# ALIGNMENT — 网页端轨迹 UI 对齐桌面

## 1. 项目上下文

- **桌面**：`FeatureTrajectoryPageWidget` + `FeatureTableModel`（5 列）+ `FeatureDiscretizerParamPanel`（策略 schema 驱动）
- **网页**：`public-fallback` 轨迹生成页已具备 PathPlan 门闩、CAD/Mesh、拾取、离散、Raw 预览；参数与表格仍简化
- **前序**：`docs/网页端轨迹对等/` 已完成 API/管线对等；本任务补 **CAD 页 UI/参数体验对等**

## 2. 原始需求

将网页版轨迹生成页面与桌面端对齐，同时完善对应功能。

对照截图：

| 桌面 | 网页现状 |
|------|----------|
| 特征表：# / 特征ID / 离散策略 / 几何摘要 / 状态 | `F1 · FaceBoundary E[] F[63]` 扁平列表 |
| 策略中文名「参数面扫描」 | 英文 ID `FaceParamSurface` |
| 完整策略参数（行间距/列间距/弦高/偏角/…） | 仅全局 `stepMm` |
| 离散参数模板 存/载/删/导入/导出 | API 有 `templates/discretize`，UI 未接 |
| 拾取状态「3D 拾取未激活」等 | 无对等文案 |
| PathPlan 标签 `face_63 · 已离散` | 较简 |

## 3. 边界确认（待拍板）

### 本轮拟纳入

1. CAD/BREP 子页：特征表 5 列 + 策略中文名 + 几何摘要 + 状态
2. `GET /api/trajectory/feature-schema?strategyId=`：透出 `featureDiscretizerAllParamFields` 合并字段
3. 策略参数面板：随策略切换动态表单（对齐 `FeatureDiscretizerParamPanel`），写入当前特征 `params`
4. FaceParamSurface 全字段可用；切换策略时灌入该策略默认值
5. 离散模板 UI（存/载/删；导入导出可二期）
6. 拾取状态文案 + PathPlan/会话状态文案贴近桌面
7. 参数变更后防抖自动再离散（可选，见决策点）

### 本轮拟排除（除非用户要求）

- Mesh 子页深度对等（视口点三角等）
- 轨迹编辑页再改版（op-schema 已动态）
- 预览坐标轴 X/Y/Z 间隔控件
- 特征右键删面/边索引（可作加分项）
- 桌面像素级 CSS 克隆（保留网页布局，信息架构对齐）

## 4. 需求理解

用户痛点不是「不能离散」，而是 **网页看不到/改不到桌面同级的策略参数与特征表语义**（例如 FaceParamSurface 仍用 EdgeChain 式 `stepMm=2`，与桌面行间距 10 / 列间距 1 等不一致）。对齐应以 **同一套 schema → 同一套 params → 同一套离散结果** 为准。

## 5. 疑问澄清（关键决策点）

见对话中提问清单。

## 6. 技术约束

- 桌面零回归；改 Host/Gateway/前端；Debug|x64 + Release|x64
- 部署 `public-fallback` → `bin\x64d\web` 与 `bin\x64\web`
- 复用 `featureDiscretizerAllParamFields` / StrategyCatalog，不另造字段表
