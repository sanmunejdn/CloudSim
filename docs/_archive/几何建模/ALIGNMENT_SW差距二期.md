# ALIGNMENT — SolidWorks 差距二期

## 项目与任务特性

- 栈：GeometricModelingPlugin → PluginSDK Host → ParametricBrepBackendData → geoalgo::SketchExtrude
- 一期已交付：平面草图、Pad/Pocket、特征再编辑、反向、UpToFace（烤平面）、多 Body MVP、回退、Undo
- 本期主线：编辑与历史 + 拉伸终止补齐（相对 SW Part 的高杠杆缺口）

## 原始需求

分析与 SolidWorks 的差距并继续完善；默认不做全量特征目录，优先让「改参 / 到面 / 3D 点选」更接近 SW。

## 差距摘要（相对 SW）

| 域 | 已有 | 主要缺口 |
|----|------|----------|
| 草图 | 线弧圆矩形 + 常用约束 | 多轮廓/投影边/相切对称等 |
| 特征 | Pad/Pocket | 圆角倒角旋转；终止条件不全 |
| 历史 | 树再编辑/回退/Undo | UpToFace 不重解；3D 拾取无面归属 |
| 拓扑 | faceIndex 弱引用可用 | 完整持久 Face/Edge 命名 |

## 边界确认（本期）

**做：**

1. MidPlane（对称）拉伸终止
2. UpToFace 存 `backendId + faceIndex`，rebuild 重解平面，失败回退烤平面
3. rebuild 建立 faceIndex→featureId，改善 3D 拾取编辑
4. 文档与原 CONSENSUS 边界修正

**不做：** Fillet/Chamfer/Revolve、ThroughAll/到顶点、多环轮廓、投影边、属性管理器重做、完整持久拓扑命名

## 需求理解

- MidPlane：总厚度 = lengthMm，双向各半；reversed 只翻轴正向
- UpToFace 活引用是弱拓扑：索引失效不炸链，降级烤平面
- 面归属表至少进程内有效；跨会话不保证（CONSENSUS 仍排除完整命名）

## 疑问澄清

无未决关键决策；按计划默认主线执行。
