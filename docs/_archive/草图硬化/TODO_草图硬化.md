# TODO — 草图硬化

## 部署

重新编译并部署：`GeometryAlgorithm.dll` + `GeometricModelingPlugin.dll`（Debug→`bin/x64d`，Release→`bin/x64`）。

## 已知局限

| 项 | 说明 |
|----|------|
| 命名参数覆盖 | 仅圆/线/椭圆 + Pad；其它特征待扩 |
| 样条控制点 | 过点拷贝为初始 poles，非 OCC Geom2d 拟合 |
| 椭圆短半轴尺寸 | 半径工具默认打 MajorRadius；短轴经参数面或 MinorRadius 约束 |
| Convert | 非圆曲线仍折线化 |

## 下一期（已写入 ROADMAP）

包 B+C 子集：圆周阵列、成角基准面、Pattern tip、startOffset/双向拉伸；其后 TopoNaming 专题。
