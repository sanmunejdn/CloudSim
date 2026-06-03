#pragma once

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
