# GeometryPlugin 开发说明

## 页面能力（V1）

- 输入源支持两种模式：`STEP 文件` / `内部后端对象（STEP/BRep）`
- 离散支持：
  - `discretizeStepToMesh`（文件）
  - `discretizeBackendToMesh`（后端对象）
  - 密度三选一：质量预设 / 目标边长(mm) / 目标三角面数（互斥）
- 求交支持：
  - 线面求交（索引输入 + 3D 点选）
  - 面面求交（索引输入 + 3D 点选）
- 结果后端化：
  - 对最近一次求交曲线可生成 `Tube` / `Ribbon` 新 mesh 后端

## 关键依赖

- `IPluginGeometryHost` 1.7.0+
  - `listComputableBackends`（仅顶层 `Model`/`BrepModel`，不含装配子零件；同 STEP 路径去重时优先 `BrepModel`）
  - `pickStepElementFromViewport`
- `PluginGeometryHostImpl` 负责：
  - 从 `DocumentPage::backendSourcePath()` 解析 STEP 路径
  - 监听 `OsgWidget::meshPickCommitted`，转换为 `PluginGeometryStepRef`

## 密度控制说明

| 模式 | UI | 算法路径 |
|------|----|----------|
| 质量预设 | Coarse/Medium/Fine | OCC 偏差离散（原逻辑） |
| 目标边长 | 边长 spin(mm) | OCC `×0.25` + 角偏差 1° → refine 到 `1.5×边长`(≤40 万) → Data `isotropicRemesh` 收口（失败保留 refine） |
| 目标面数 | 面数 spin | OCC 相对 deflection 二分，约 ±15% |

**刻意不做**：偏粗 OCC 基网格、`isotropicRemeshProgressive`、remesh 失败再最长边细分兜底。

## 验收清单

1. Geometry 页面可切换 `STEP 文件` 与 `内部后端对象` 两种输入源
2. 选择后端对象后执行离散，成功生成新 mesh backend 并在树中选中
2b. 密度模式可切换并传到 Host；边长模式状态栏显示 `avgEdge`
3. 线面求交支持：
   - 直接输入 edge/face 索引
   - 点击「点选边」「点选面」后在 3D 视图选择
4. 面面求交支持：
   - 直接输入 face1/face2 索引
   - 点击「点选 F1」「点选 F2」后在 3D 视图选择
5. 求交完成后可点击「生成管状网格」或「生成带状网格」得到新后端对象
6. `plugin.json` 的 `minHostVersion` 为 `1.7.0`，插件可正常加载
