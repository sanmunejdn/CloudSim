#ifndef WIDGET_WIDGETDOCUMENTACCESS_H
#define WIDGET_WIDGETDOCUMENTACCESS_H

/// @file WidgetDocumentAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Widget/Host 经 IRenderView::widget() 取得 OsgWidget

#include "DocumentHost.h"
#include "IRenderView.h"
#include "OsgWidget.h"

/// Widget/Host 经 IRenderView::widget() 取得 OsgWidget
inline OsgWidget* widgetOsgFromPage(cloudsim::host::DocumentHost* page)
{
	if (!page)
	{
		return nullptr;
	}
	return qobject_cast<OsgWidget*>(page->render().widget());
}

#endif // WIDGET_WIDGETDOCUMENTACCESS_H
