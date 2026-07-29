# ALIGNMENT — SW 差距续期（造型收尾包之后）

## 1. 项目上下文

- 插件：`GeometricModelingPlugin`；几何：`GeometryAlgorithm`；历史：`ParametricBrepBackendData`
- 宿主 ABI：`0x00012600`（1.38.0）
- 装配体 / 曲面模块 / 全拓扑命名：仍明确不做

## 2. 上一里程碑已交付（基线已变）

| 层 | 已有 |
|----|------|
| Kind | Sketch/Pad/Pocket/Sweep(+Cut)/Fillet/Chamfer/Revolve(+Cut)/LinearPattern/Mirror3D/Loft(+Cut)/Shell |
| 草图 | 相切/对称/中点、投影边、多环+孔岛导出、Pad 孔贯通 |
| UI | Ribbon + 侧栏预览提交；扫描「点选轮廓/路径」（特征树） |

## 3. 原清单剩余（本续期范围候选）

### P1 未完

- 椭圆 / 多边形 / 槽口
- 等距 Offset
- 转换实体 Convert（面边→草图，投影边已有轻量版）
- 面上草图再编辑/正视稳固（正视已挂，再编辑体验仍弱）

### P2 未完

- 扫描增强：引导线、扭转、薄壁、模型边作路径
- 独立拔模 Draft
- 筋 Rib / 孔向导
- 拉伸终止：到顶点 / 到面偏移

### P3 未动手

- 基准面/轴/坐标系/参考点
- 边/面稳定 id（弱拓扑二期）
- 特征级 Suppress UI
- PM 级属性面板
- ThroughAll/拔模失败策略硬化

### 已知债务（影响「像 SW」观感）

- LinearPattern/Mirror3D 作用于 **tip 整实体**，非「选特征阵列」
- 双击再编辑多数不回填参数
- 圆角边索引跨 rebuild 仍弱引用

## 4. 原始需求理解

在「造型收尾包」已缩小能出零件差距后，继续按杠杆收缩与 SolidWorks Part 的差距；优先 **每天都用、且不依赖装配** 的能力。

## 5. 疑问澄清（待决策）

见下方「关键决策点」。
