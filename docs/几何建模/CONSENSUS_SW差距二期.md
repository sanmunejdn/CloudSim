# CONSENSUS — SolidWorks 差距二期

## 需求描述

在一期几何建模 MVP 上补齐：对称拉伸（MidPlane）、UpToFace 可重解面引用、3D 面拾取到正确特征。

## 验收标准

1. **MidPlane**：新建 Pad 选「对称」；预览/提交厚度正确；再编辑仍为 MidPlane；history JSON 含 `MidPlane`
2. **UpToFace 活引用**：绑定面 A 后，上游改参使 A 平移再 Rebuild，拉伸高度跟随；faceIndex 失效时仍用烤平面出实体
3. **面归属拾取**：两段先后 Pad 上点不同面，分别打开对应特征编辑；无法归属时回退末特征启发
4. Host + 插件编译通过

## 技术方案

| 项 | 方案 |
|----|------|
| MidPlane | 枚举贯通；棱柱双向各 `L/2` |
| 活引用 | 特征存 `upToFaceBackendId` + `upToFaceIndex` + 烤平面回退；rebuild 内 OCC 取面平面 |
| 面归属 | rebuild 每步 Pad/Pocket 后 Face 差集 → 内存 map；`pickParametricFeatureForEdit` 查表 |

## 技术约束

- 插件不链 GeometryAlgorithm/Data/OCC；面解析在 Data/Host
- Host vtable 仅必要时追加；优先用现有 API
- 弱拓扑，非完整命名系统

## 任务边界

本期不做：圆角等特征目录、ThroughAll/到顶点、多环轮廓、投影边、完整拓扑命名、属性管理器大改。
