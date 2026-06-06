# CloudSim UI 图标资源

CloudSim 全前端共享图标，嵌入 `cloudsim_icons.qrc`，由 `CloudSimUiAssets.dll` 导出 `UiIcons` / `UiIconDecorators` API。

## 视觉规范

- 风格：Material Outlined 几何近似（单色线框）
- Light 前景 `#424242`，Dark 前景 `#E0E0E0`
- 尺寸：16px / 24px 透明 PNG

## 重新生成 PNG

```powershell
python CloudSim/tools/ui-icons/generate_icons_pillow.py
```

网络可用时亦可尝试从 Google Material 仓库导出：

```powershell
python CloudSim/tools/ui-icons/export_material_symbols.py
```

## 许可

图标几何由项目脚本生成；若改用 [Material Symbols](https://fonts.google.com/icons)，遵循 Apache License 2.0。
