# TODO — 三维模型 → 二维工程图

## 已交付（P1–Q17）

- 多视图 HLR、剖视/局部、半剖简版、切线驱动切面、细节圆可编
- 草图绘制与修改：Trim/Extend/Offset/Fillet/Chamfer（含弧）/Break/Join/**Stretch**；矩形/环阵（**含草图实体**）
- 线性/角度/半径/直径、连续/基线尺寸；弱关联边键
- **粗糙度**与**形位公差框**（轻量）；DXF 以 TEXT/几何近似
- 图层 ByLayer/ByBlock、图块/属性上图、打印预览、CTB
- DXF 圆/弧/块属性往返；选择/捕捉交互高亮；右栏「属性」
- 产品帮助：`help/{zh,en}/drawing.html`（由 `_generate_manual.py` 生成）

## 后置（不做于本阶段）

- DWG 自研、动态块、外参、LISP、完整 AutoCAD 命令集
- 阶梯剖、真 DIMASSOC→B-rep、PlaneGCS 全图约束、任意用户轴测相机
- DXF 原生 `DIMENSION` / `TOLERANCE` 完整实体

## 待人工点验

1. 帮助 → 工程图纸：连续/基线、Break/Join/Stretch、阵列草图、ByBlock、半剖、粗糙度/形位公差、属性面板（中英）
2. Stretch 窗选顶点后位移；线-弧倒圆/倒角；草图阵列
3. 粗糙度与公差框上图、保存重开、DXF 可见文字
4. Debug\|x64 与 Release\|x64 插件加载正常
