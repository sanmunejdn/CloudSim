# GeometricModelingPlugin

独立 SDK 插件「几何建模」（`com.cloudsim.geomodeling`）：宿主顶栏切换工作区、中央页嵌入 3D、选面/基准面草图、Pad/Pocket 及扫描/圆角/旋转等实体特征、特征树与 Undo。

## 构建

Visual Studio 生成 **GeometricModelingPlugin**，须同时编 **Debug|x64** 与 **Release|x64**：

| 配置 | 产物目录 |
|------|----------|
| Debug\|x64 | `bin/x64d/plugins/com.cloudsim.geomodeling/` |
| Release\|x64 | `bin/x64/plugins/com.cloudsim.geomodeling/` |

产物含 `GeometricModelingPlugin.dll` + `plugin.json`。

依赖 Host ABI **≥ 1.48.0**（`0x00013000`）。插件拒绝加载更低版本宿主。

工程使用 **C++20**（PlaneGCS）；依赖链常见为：CloudSimPluginSDK → GeometryAlgorithm → Data → CloudSimHost → GeometricModelingPlugin。

## 使用

1. 打开含 B-rep / 可建模文档  
2. 宿主顶栏工作区切换到 **几何建模**（非顶层菜单；见 WorkspaceModeSwitcher）  
3. **新建草图** → 拾取平面面或原点平面 → 绘制并 **求解** → **拉伸 / 切除** 或其他特征  
4. 切回 **主程序** 或其它工作区退出建模模式  

Ribbon 分组：草图（绘制/引用）、标注（尺寸/几何约束）、特征（实体 + 重建）、撤销/重做。

## 功能一览

| 类别 | 能力 |
|------|------|
| 工作区 | 模式进入/退出、Ribbon、特征树、Undo/Redo、工程侧车 + Body history 同步 |
| 草图绘制 | 直线/圆弧/圆/矩形/椭圆/多边形/槽口/样条、构造线、修剪/镜像/删除 |
| 尺寸与约束 | 长度/距离/半径/角度；H/V/重合/平行等 + PlaneGCS 求解 |
| 草图引用 | 投影边、转换实体、等距 |
| 实体特征 | Pad/Pocket（多种终止 + 拔模角）、Sweep/SweepCut、Fillet/Chamfer、Revolve、LinearPattern、Mirror3D、Loft、Shell、Draft、DatumPlane |

完整清单、MVP 局限与下一期路线图见开发文档。

## 文档

- 活跃：[docs/几何建模/](../../../docs/几何建模/)（功能清单、架构、路线图）  
- 历史 6A：[docs/_archive/几何建模/](../../../docs/_archive/几何建模/)  
- 近期专题：`docs/_archive/SW差距续期/`、`docs/_archive/硬化基准面/`、`docs/_archive/特征史AI/`、`docs/_archive/WorkspaceModeSwitcher/`
