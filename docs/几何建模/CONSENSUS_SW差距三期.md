# CONSENSUS — SolidWorks 差距三期

## 验收标准

1. Pocket/Pad（合并）可选「贯通」；预览与提交切穿基实体；history JSON 含 `ThroughAll`
2. 无基实体时选贯通 → 明确报错，不成实体
3. Pocket 时「新建实体」不可用；活动实体下拉文案为切除目标；空目标不可开预览
4. Host + 插件 Debug 编译通过（minHostVersion 1.30.0）

## 技术方案

| 项 | 方案 |
|----|------|
| ThroughAll | 基实体 AABB 沿拉伸方向投影最大正距离 + 余量 |
| Pocket 目标 | 复用 `activeBodyId` / `targetParametricBackendIdUtf8`，UI 强制与文案 |
