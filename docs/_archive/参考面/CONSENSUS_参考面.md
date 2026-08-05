# CONSENSUS — 参考面最小闭环 + 关联/二次编辑

## 决策

用户选择：**C（A+B）+ D**。

| 代号 | 内容 | 优先级 |
|------|------|--------|
| **C-A** | 「新建草图」视口可点选已创建用户参考面 | P1 必做 |
| **C-B** | 等距创建支持源 = 原点基面（XY/XZ/YZ）或模型平面面 | P1 必做 |
| **D-1** | 参考面持久化源引用 + 偏移/角度；侧栏可二次编辑并应用 | P2 |
| **D-2** | 关联驱动：源面/基面仍有效时，重建或改参后重算参考面帧 | P2 |

交付顺序：**先完成 P1（C），再做 P2（D）**；同一专题文档与验收，不拆成无关项目。

## 明确需求

1. **创建**：Ribbon「基准面」→ 等距模式可选原点基面或模型平面面 → 输入偏移 → 特征树出现 DatumPlane + overlay。
2. **开草图**：Ribbon「新建草图」可点选该参考面；树上双击仍可用。
3. **二次编辑（D）**：选中/双击参考面打开侧栏（对齐 Extrude 页模式），可改偏移（成角改角度）、可重选源；确认后更新 `plane` 与 overlay。
4. **关联（D）**：保存 `source`（OriginPlane 索引 **或** Face `backendId`+`faceIndex`）+ `offsetMm`（成角另存 hinge/angle）；Body sync / Rebuild / 应用参数后按源重算 `plane`。源失效时保留上次烘焙帧并警告。

## 验收标准

### P1（C）

| # | 标准 |
|---|------|
| 1 | 等距：点选 XY/XZ/YZ 之一 → 输入偏移 → 树中有 DatumPlane，视口有半透明面 |
| 2 | 等距：点选模型平面面 → 行为与现网一致且仍可用 |
| 3 | 新建草图：点选用户参考面 overlay → 进入草图且平面一致 |
| 4 | 新建草图：原点基面、模型面原有路径不回归 |
| 5 | Debug\|x64 + Release\|x64 插件（及若改 Host 则 Host）编译通过 |

### P2（D）

| # | 标准 |
|---|------|
| 6 | DatumPlane JSON 含源类型与偏移（或成角字段）；重开工程可编辑 |
| 7 | 侧栏改偏移 → Apply → overlay/plane 更新 |
| 8 | 父模型面仍在时 Rebuild/sync 后偏移面位置与源一致 |
| 9 | 源面丢失：不崩溃；日志警告；保留上次 plane |

## 技术方案（摘要）

### P1

- **C-B**：`onDatumPlane` 等距分支改为「原点平面拾取 ‖ 模型面拾取」并行（复用 Host 原点拾取或插件内二次选择对话框：先选「基面/模型面」再拾取）。优先复用 `pickOriginSketchPlane` 的并行语义，取得平面后乘偏移。
- **C-A**：`onNewSketch` / `pickOriginSketchPlane` 增加用户 Datum 命中：插件维护的 overlay 四边形与射线求交，或 Host 提供 `pickUserSketchPlanes(planes[])`。优先 **插件侧在 pick 回调前/并行做 overlay 命中**（少动 ABI）；若必须进 Host 则 bump ABI。

### P2

- `GeomodelingFeature` 增字段（命名示意）：
  - `datumSourceKind`: None / OriginPlane / Face /（成角已有 hinge）
  - `datumOriginPlaneIndex`（0–2）
  - `datumFaceBackendId` + `datumFaceIndex`
  - `datumOffsetMm`
- 侧栏 `m_pageDatum`：偏移 spin、重选源、Apply/Cancel；双击 Datum **先开侧栏**（不再直接开草图）；侧栏提供「在此面开草图」按钮保留原路径。
- `reevaluateDatumPlanes(page)`：在 `syncFeaturesFromBody` / Rebuild 后调用。

### 约束

- DatumPlane **仍不进** Parametric tip（侧车策略不变），除非后续单独立项。
- 注释纯中文、少而聚焦 Why；外科手术式改动。
- 若改 Host ABI：宿主与插件同版本。

## 边界限制

**不做（本专题）：**

- 参考轴 / 坐标系
- 多约束参考面全集（点+线、两面夹角新模式等；成角/三点保持现状）
- 把 Datum 写入 Parametric tip / AI compose 全量
- SolidWorks 级完整 PropertyManager 视觉

**草图跟随（待下一问确认）：** 改偏移后，**已挂在该参考面上的草图**是否同步更新 `plane` 并重投影 profile——见下方未决项。

## 已确认（原未决）

**草图跟随 = D-S1**：Sketch 存 `datumPlaneId`；改参/重建时更新该草图 `plane`，并由 UV 文档重导出 `profileXyzMm`，再 `rebuildDownstreamAfterSketch`。

## 假设（已采纳）

1. P1 与 P2 同一 `docs/参考面/` 专题，按 TASK 顺序交付。
2. 双击 Datum 在 P2 改为打开侧栏；侧栏提供「在此面开草图」。
3. 三点模式不改；成角 P2 可编角度；等距主路径必须关联；成角源面+边关联为同字段增强（有源则重算）。
