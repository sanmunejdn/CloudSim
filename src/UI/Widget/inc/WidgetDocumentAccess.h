#ifndef WIDGET_WIDGETDOCUMENTACCESS_H
#define WIDGET_WIDGETDOCUMENTACCESS_H

/// @file WidgetDocumentAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Widget/Host 经 DocumentHost::osgView() 取得视口窄接口

#include "DocumentHost.h"
#include "IOsgWidgetView.h"

/// 插件等存量路径：与 Host 内 osgWidgetFrom 等价
inline IOsgWidgetView* widgetOsgFromPage(cloudsim::host::DocumentHost* page)
{
	if (!page)
	{
		return nullptr;
	}
	return page->osgView();
}

#endif // WIDGET_WIDGETDOCUMENTACCESS_H
