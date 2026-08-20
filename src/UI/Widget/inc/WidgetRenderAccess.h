#ifndef WIDGET_WIDGETRENDERACCESS_H
#define WIDGET_WIDGETRENDERACCESS_H

/// @file WidgetRenderAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
