# ACCEPTANCE — SolidWorks 差距二期

## 完成情况

| 项 | 状态 | 说明 |
|----|------|------|
| 文档 ALIGNMENT/CONSENSUS | 已完成 | `docs/几何建模/ALIGNMENT_SW差距二期.md` 等 |
| MidPlane 贯通 | 已完成 | Algo/Data/SDK/插件 UI「对称」 |
| UpToFace 活引用 | 已完成 | `upToFaceBackendId`+`upToFaceIndex`；rebuild 重解，失败烤平面 |
| 面归属拾取 | 已完成 | `mergeFaceOwnershipByTShape` + `featureIdForFace` |
| Debug 编译 | 已通过 | GeometryAlgorithm → Data → SDK → CloudSimHost → GeometricModelingPlugin |

## 手测清单（需在 UI 确认）

1. Pad 选「对称」，改长度预览/提交；再编辑仍为 MidPlane；history 含 `"MidPlane"`
2. UpToFace 绑定面后改上游长度 Rebuild，高度跟随；删面后仍能靠烤平面出实体
3. 两段 Pad 后「视口点选编辑」：点不同面打开对应特征；无归属时弹菜单

## Host 版本

插件要求 **1.29.0**（`0x00011D00`）。
