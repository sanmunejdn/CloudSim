# ALIGNMENT — 实体扫描特征

## 需求

SolidWorks 式实体扫描（凸台/切除）：轮廓草图 + 路径草图 → MakePipe → Fuse/Cut 进 Parametric Body。

## 边界

**做：** Sweep / SweepCut；路径草图 Line+Arc 成 wire；轮廓闭合折线；预览/提交/再编辑/下游重建。

**不做：** 开曲面、引导线、扭转、模型边路径、多轮廓、薄壁。

## 决策

- 两 kind：`Sweep` / `SweepCut`（对称 Pad/Pocket）
- 核：`BRepOffsetAPI_MakePipe`（参考 OneCAD，不链 ported）
- 路径：`pathXyzMm` 折线烤入特征；真弧边在算法侧由折线近似（MVP）
