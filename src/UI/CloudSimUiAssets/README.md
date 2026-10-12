# CloudSim UI 图标资源

CloudSim 全前端共享图标，嵌入 `cloudsim_icons.qrc` / `cloudsim_logo.qrc`，由 `CloudSimUiAssets.dll` 导出 `UiIcons` / `UiIconDecorators` / `AppIcon` API。

## 应用 Logo

- 真源 SVG：`resources/logo/cloudsim_logo.svg`（Qt 可栅格化；无 SVG filter）
- 设计全稿：`resources/logo/cloudsim_logo_full.svg`（含 glow，供设计工具）
- 嵌入 PNG：`light|dark/cloudsim_logo_{16,24,32,48,64,128,256}.png`（当前浅深同图）

重新生成 Logo PNG：

```powershell
powershell -ExecutionPolicy Bypass -File CloudSim/tools/ui-icons/generate_logo_from_svg.ps1
```

## UI 线框图标视觉规范

- 风格：Material Outlined 几何近似（单色线框）
- Light 前景 `#424242`，Dark 前景 `#E0E0E0`
- 逻辑尺寸：16px（Small）/ 24px（Medium）
- 位图：另含 32 / 48（`@2x`），供高分屏

## 重新生成 UI 线框 PNG

```powershell
python CloudSim/tools/ui-icons/generate_icons_pillow.py
```

网络可用时亦可尝试从 Google Material 仓库导出：

```powershell
python CloudSim/tools/ui-icons/export_material_symbols.py
```

## 许可

Logo 为项目自有；UI 线框图标由项目脚本生成。若改用 [Material Symbols](https://fonts.google.com/icons)，遵循 Apache License 2.0。
