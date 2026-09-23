# CloudSimUiAssets 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `CloudSimUiAssets.dll` |
| 职责 | 共享 UI 图标资源（`cloudsim_icons.qrc`）；导出 `UiIcons` / `UiIconDecorators` |

## 2. 边界

- **负责**：图标嵌入与按主题取色 API
- **不负责**：业务控件布局

## 3. 已交付能力

- Material Outlined 风格线框图标；Light/Dark 前景色
- 逻辑尺寸 16/24；高分屏 32/48

## 4. 再生图标

见同目录 [README.md](README.md)（`tools/ui-icons/` 脚本）。
