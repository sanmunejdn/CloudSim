# GeometricModelingPlugin

独立 SDK 插件「几何建模」：菜单进入、中央页嵌入 3D、选面约束草图、Pad/Pocket、特征树与 Undo。

## 构建

Visual Studio 生成 **GeometricModelingPlugin**（Debug|x64）。产物：

`bin/x64d/plugins/com.cloudsim.geomodeling/GeometricModelingPlugin.dll` + `plugin.json`

依赖 Host ABI **≥ 1.24.0**（`CLOUDSIM_PLUGIN_HOST_VERSION 0x00011800`）。进入模式后命令在菜单下 Ribbon（草图：新建/直线/圆弧/圆/矩形/结束；特征；编辑）。Pad/Pocket 写入同一 `ParametricBrepModel` Body。

## 使用

1. 打开含 B-rep 的文档  
2. 菜单 **几何建模 → 进入几何建模**  
3. **新建草图** → 拾取平面面 → **求解** → **拉伸 Pad** / **裁剪 Pocket**  
4. **返回三维场景**

## 文档

见 [`docs/几何建模/`](../../../docs/几何建模/)。
