#ifndef CLOUDSIMHOST_IOSGWIDGETVIEW_H
#define CLOUDSIMHOST_IOSGWIDGETVIEW_H

/// @file IOsgWidgetView.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 共享 Host 源依赖的 OsgWidget 窄接口（桌面/Headless 各自实现）
/// @details 由 SceneOps / PickObserve / Toolbar / Overlay 切面多继承合成；
///          Qt 场景信号见 IViewportSceneSignals，不并入本接口。

#include "IViewportOverlay.h"
#include "IViewportPickObserve.h"
#include "IViewportSceneOps.h"
#include "IViewportToolbar.h"

/// 共享库抽取前置：共享源只依赖此接口，不直接 include 真/桩 OsgWidget.h
/// vtable 仅允许在末尾追加新方法，禁止插入或改签名（切面内各自追加）
class IOsgWidgetView : public IViewportSceneOps,
					   public IViewportPickObserve,
					   public IViewportToolbar,
					   public IViewportOverlay
{
public:
	using AnnotationSnapshot = IViewportOverlay::AnnotationSnapshot;
	using SketchSupportExtraPlane = IViewportOverlay::SketchSupportExtraPlane;
	using OriginPlanePickedFn = IViewportOverlay::OriginPlanePickedFn;
	using SketchPlaneInputHandler = IViewportOverlay::SketchPlaneInputHandler;

	~IOsgWidgetView() override = default;
};

#endif // CLOUDSIMHOST_IOSGWIDGETVIEW_H
