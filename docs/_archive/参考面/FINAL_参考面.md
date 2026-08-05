# FINAL — 参考面

## 交付摘要

在现有 DatumPlane 上补齐 **C（新建草图点选用户面 + 等距源含基面）** 与 **D（关联源 + 参数二次编辑 + D-S1 草图跟随）**。

## 关键变更

- Host ABI **1.49.0**（`0x00013100`）：`pickSketchSupportPlane`
- OsgWidget：用户候选面与基面/模型面深度裁决（index≥100）
- FeatureDocument：`datumSource*`、`datumOffsetMm`、Sketch.`datumPlaneId`
- Plugin：等距/新建草图接线；`reevaluateDatumPlanes`；参数页改偏移/角度

## 局限

- 成角基准面创建时未写入 Face 源（改角度需已有源或仅存角度）
- 二次编辑用命名参数页，非独立 PropertyManager
- Datum 仍不进 Parametric tip
