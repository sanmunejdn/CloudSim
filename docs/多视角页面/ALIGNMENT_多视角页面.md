# ALIGNMENT — 多视角页面

## 原始需求

允许多台相机视角与页面观察：文档内分屏 + 可另开视角窗；场景自由 Orbit。

## 项目上下文

- 现状：一 `DocumentPage` → 一 `OsgWidget` / `osgViewer::Viewer`
- 入口：`DocumentHost` + `mountFlavorOsgViewport`（Desktop）
- 约束：勿用 `QStackedWidget` 包 OpenGL；Headless 无 OSG

## 边界

- 做：1/2/4 分屏、浮动视角窗、独立 Trackball、方向预设、活动格高亮
- 不做：工业相机画面、CompositeViewer 重写、卫星格完整拾取

## BugTracker

#35
