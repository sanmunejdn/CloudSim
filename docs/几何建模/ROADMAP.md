# ROADMAP — 已知债与下一期落地方案

本期（文档盘点）**不实现**下列功能代码；本文锁定四包内容、改动面与验收，便于后续单独立项。

## 1. 已知 MVP 债（盘点）

来源：`docs/SW差距续期/TODO_*`、`docs/硬化基准面/TODO_*`，以及 FEATURES 中的局限列。

| 项 | 说明 |
|----|------|
| 椭圆约束 | 可绘可出体，未进 PlaneGCS |
| Convert 圆弧 | 投影仍可能折线化 |
| Offset | 复杂尖角 / 孔环边界仍可能失败 |
| 扫描扭转 | 轮廓绕起点切向预转，非 PipeShell 扭转律 |
| Vertex | 边端点吸附，非 TopExp 顶点索引 |
| Pattern tip | 源特征与 Pattern 间中间特征可能被 tip 替换抹掉 |
| 成角基准面 | 未做（等距/三点已有） |
| 视口点选 | Sweep 等仍偏 combo；路径/轮廓点选未完全硬化 |

## 2. 明确不做（产品边界）

- 装配 / 配合
- 曲面 Body / 曲面上草图
- 工程图双向尺寸（属工程图专题）
- 完整 FreeCAD/SW 级持久拓扑命名引擎（仅允许「命名起步」）
- 引导线扫描的薄壁变体、Rib、孔向导（除非单独立项）

## 3. 推荐实现顺序

```text
草图硬化 → 特征升级 → 交互与拓扑（点选/Suppress）→ 扫描拉伸深化 → Face/Edge 命名起步
```

理由：草图质量喂给所有特征；Pattern/基准面不依赖 PipeShell；点选改善体验后再上引导线与双向拉伸；拓扑命名依赖稳定引用场景，放最后起步。

## 4. 包 A — 草图硬化

| 项 | 落点 | 要点 |
|----|------|------|
| 椭圆进 GCS | `SketchConstraintSolver` + `third_party/planegcs` | 半径/角度类约束；导出仍可离散供 Pad |
| Convert 保圆弧 | Host 投影路径 + `SketchGeom` | 共面圆弧写 `SkArc`；失败回退折线并提示 |
| 样条控制点 / GCS | `SketchEditSession` + planegcs | 先控制点拖拽；BSpline 约束按能力渐进 |

**改动工程**：GeometricModelingPlugin（必）；GeometryAlgorithm / Host（若投影返回几何类型变化则需）。  
**ABI**：仅当 Host 投影 API 扩语义时 bump。  
**验收**：Ribbon 椭圆可加半径约束并求解；Convert 圆孔边为圆弧实体；样条可拖控制点后 Pad 出体；Debug+Release 插件通过。

## 5. 包 B — 特征升级

| 项 | 落点 | 要点 |
|----|------|------|
| Pattern tip 语义修正 | `ParametricBrepBackendData` + Pattern rebuild | 明确 `tipAfterFeature` 边界，避免中间特征被抹掉 |
| 特征级线性阵列硬化 | 已有 `sourceFeatureId` | 与 tip 修正一并验收；失败提示可读 |
| 圆周阵列 | 新 `CircularPattern` kind + `SketchPattern` | 轴 + 角度跨度 + 数量；history JSON 往返 |
| 成角基准面 | DatumPlane 创建路径 | 参考面 + 角度；树节点 + overlay |

**改动工程**：GeometryAlgorithm、Data、CloudSimPluginSDK/Host（圆周阵列参数）、GeometricModelingPlugin。  
**ABI**：圆周阵列 / 新参数 → 须 bump；仅 tip 修正可能无需新 API。  
**验收**：中间特征 + Pattern 重建形状正确；圆周阵列可删回退；成角基准面可新建草图；双配置编译 + 存盘重开。

## 6. 包 C — 扫描 / 拉伸深化

| 项 | 落点 | 要点 |
|----|------|------|
| PipeShell 真扭转 | `SketchSweep` | 替换轮廓预旋转；侧栏扭转角语义对齐 OCC |
| 引导线扫描 | Sweep params + Host | 额外引导曲线；失败可读，不做薄壁 |
| 双向独立拉伸长度 | Pad/Pocket 参数 / rebuild | 两方向长度独立（非仅 MidPlane） |

**改动工程**：GeometryAlgorithm、Data history schema、SDK/Host、Plugin 侧栏。  
**ABI**：引导线、双向长度参数 → 须 bump。  
**验收**：扭转随路径律变化可观测；引导线失败有提示不静默坏体；双向 Pad 历史往返；双配置全链路编译。

## 7. 包 D — 交互与拓扑

| 项 | 落点 | 要点 |
|----|------|------|
| 视口点选硬化 | SolidFeatures + Host pick | Sweep 轮廓/路径、Fillet 边等少依赖 combo |
| 特征 Suppress | `GeomodelingFeature.suppressed` 已有字段 | 贯通 rebuild 跳过 + 特征树 UI |
| Face/Edge 命名起步 | Parametric / Host | 稳定 id 或弱命名表；先服务 UpToFace/Fillet；**非**完整拓扑引擎 |

**改动工程**：Plugin（必）；Host/Data（命名表与 Suppress 进 history 时）。  
**ABI**：命名查询/绑定 API → 须 bump；纯 UI 点选复用现有 pick 可不 bump。  
**验收**：Sweep 可点选路径边完成；Suppress 后 tip 跳过该特征；UpToFace 跨重建弱引用改善有用例；双配置编译。

## 8. 跨包通用验收

1. Debug\|x64 与 Release\|x64：SDK / GeometryAlgorithm / Data / Host / GeometricModelingPlugin 均通过  
2. 历史 JSON 往返后形状可恢复；弱引用失败明确报错  
3. 注释纯中文、少而聚焦 Why；改动保持外科手术式  
4. 不把 DLL 拷到非工程 `OutDir` 冒充生成结果  

## 9. 与历史文档关系

- `_archive/几何建模/` 中早期「不做圆角/扫描」等表述 **已过时**；以 FEATURES + 本 ROADMAP 为准  
- 已交付增量细节仍以 `SW差距续期/`、`硬化基准面/`、`特征史AI/` 的 FINAL/TODO 为准  
- 开干某一包时，再按 6A 新建该包的 ALIGNMENT/CONSENSUS（勿改写 archive 旧 CONSENSUS）
