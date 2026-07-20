#ifndef WIDGET_WIDGETDOCUMENTACCESS_H
#define WIDGET_WIDGETDOCUMENTACCESS_H

/// @file WidgetDocumentAccess.h
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
