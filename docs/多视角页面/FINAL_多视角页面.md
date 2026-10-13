# FINAL — 多视角页面

## 交付摘要

文档内 1/2/4 分屏 + 最多 2 个浮动视角窗；共享场景根、各格独立 Trackball；拾取/工具栏仍走 Primary。

## 主要改动

- `SatelliteOsgView` / `MultiViewShell` / `DetachedViewWindow`（DesktopViewport）
- `DocumentHost` 多视角 API + flavor hooks（Headless no-op）
- 视图菜单「多视角」、视口工具栏「田」按钮；中英 i18n；QSettings 存 layout
- 6A 文档：`docs/多视角页面/`
- BugTracker **#35**

## 编译

| 工程 | Debug\|x64 | Release\|x64 |
|---|---|---|
| CloudSimHostDesktopViewport | OK | OK |
| CloudSimHost | OK | OK |
| Widget | OK | OK |
| CloudSimHostHeadless | OK | OK |
