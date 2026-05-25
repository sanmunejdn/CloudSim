#pragma once

#include "DocumentPage.h"
#include "IRenderView.h"
#include "OsgWidget.h"

/// Widget 侧经 IRenderView::widget() 取得 OsgWidget，与 Host osgWidgetFrom 对齐
inline OsgWidget* widgetOsgFromPage(DocumentPage* page)
{
	if (!page)
	{
		return nullptr;
	}
	return qobject_cast<OsgWidget*>(page->render().widget());
}
