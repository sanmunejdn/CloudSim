# CONSENSUS_网页端主程序壳对等

桌面「主程序」=`modeId=""` 的 MainWindow；网页对应默认工作区 `scene3d`。

## 本次范围（壳 + 续补）

| 能力 | 桌面 | 网页落地 |
|------|------|----------|
| 打开点云 | 文件菜单 | 文件菜单 → `purpose=pointcloud` |
| 左右面板显隐 | 视图菜单 | 视图菜单 + 视口条；localStorage |
| 重置布局 | 视图菜单（桌面未接线） | 恢复默认宽与可见 |
| 视口条 | 选择/聚焦/线框/截图/面板 | 同左（主视图保留） |
| Gizmo 物体系/世界系 | 视图菜单 | `TransformControls.setSpace` |
| 关于 | 帮助对话框 | 简易关于对话框 |
| 插入配合 | Mate 面板 + AssemblyMateApply | `POST /api/assembly/mate` + 插入→配合对话框 |
| 未保存提示 | 关页询问 | 新建/打开前 confirm（SSE 脏标记） |

## 仍不做

- 真多文档、点/线/面拾取全套（视图菜单级）、场景层级树
- 插件工作区加厚、PLC/相机、AI、外部轴/通讯页
