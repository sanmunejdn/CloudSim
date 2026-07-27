# ALIGNMENT — SolidWorks 差距三期

## 原始需求

在二期（MidPlane / UpToFace 活引用 / 面归属）之上继续优化。

## 本期边界

**做：**

1. 拉伸终止 **ThroughAll（贯通）**：相对基实体包围盒沿法向求贯通长度
2. **Pocket 目标实体**：切除明确绑定活动 Body 下拉；禁止新建实体；文案标明「切除目标」

**不做：** Fillet、到顶点、多环轮廓、投影边、完整拓扑命名

## 约定

- ThroughAll 必须有基实体（Pocket 必有；Pad 合并到已有 Body 时可用）
- 无基时 ThroughAll 报错，不静默退化成 Blind
