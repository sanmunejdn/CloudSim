#ifndef WIDGET_WIDGETDOCUMENTACCESS_H
#define WIDGET_WIDGETDOCUMENTACCESS_H

/// @file WidgetDocumentAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief DocumentHost 视口分面 helper；勿再引入 fat IOsgWidgetView*

#include "DocumentHost.h"
#include "IViewportOverlay.h"
#include "IViewportPickObserve.h"
#include "IViewportSceneOps.h"
#include "IViewportToolbar.h"

inline IViewportSceneOps* widgetSceneOpsFromPage(cloudsim::host::DocumentHost* page)
{
	return page ? page->sceneOps() : nullptr;
}

inline IViewportPickObserve* widgetPickObserveFromPage(cloudsim::host::DocumentHost* page)
{
	return page ? page->pickObserve() : nullptr;
}

inline IViewportOverlay* widgetOverlayFromPage(cloudsim::host::DocumentHost* page)
{
	return page ? page->overlay() : nullptr;
}

inline IViewportToolbar* widgetToolbarFromPage(cloudsim::host::DocumentHost* page)
{
	return page ? page->toolbar() : nullptr;
}

#endif // WIDGET_WIDGETDOCUMENTACCESS_H
