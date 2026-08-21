# FINAL — 视口拾取重构

## 交付摘要

完成视口拾取框架落地：`ViewportInteraction`（Engine / Hit / Policy / Tool·Overlay 适配器 / Controller / Session），`OsgWidget` 经 Controller 分发事件，旧 `set*Mode` 保留为门面。

## 主要路径

| 路径 | 说明 |
|------|------|
| `Widget/inc|source/ViewportInteraction/**` | 新框架 |
| `OsgWidget` | setupInteractionController、mode→tool、Session API |
| `WidgetSceneSignalWiring` | Session 优先于全局 mesh 回调 |
| `CloudSimHost.vcxproj` | 同步编译 Engine/Controller |

## 编译

Debug|x64 已通过；Release|x64 见 `_build_release.txt`。
