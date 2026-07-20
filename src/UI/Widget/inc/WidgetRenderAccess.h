#ifndef WIDGET_WIDGETRENDERACCESS_H
#define WIDGET_WIDGETRENDERACCESS_H

/// @file WidgetRenderAccess.h
/// @brief WidgetRenderAccess 接口

#include "DocumentHost.h"
#include "IRenderView.h"

inline cloudsim::core::IRenderView* renderViewFromPage(cloudsim::host::DocumentHost* page)
{
	return page ? &page->render() : nullptr;
}

inline QWidget* renderWidgetFromPage(cloudsim::host::DocumentHost* page)
{
	cloudsim::core::IRenderView* rv = renderViewFromPage(page);
	return rv ? rv->widget() : nullptr;
}

#endif // WIDGET_WIDGETRENDERACCESS_H
