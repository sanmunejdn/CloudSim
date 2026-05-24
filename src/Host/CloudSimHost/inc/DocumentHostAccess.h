#pragma once

#include "DocumentHost.h"
#include "IRenderView.h"
#include "OsgWidget.h"

namespace cloudsim::host {

/// Host 内 OSG 唯一转换点：经 IRenderView::widget() 取得 OsgWidget。
inline OsgWidget* osgWidgetFrom(DocumentHost& host)
{
	return qobject_cast<OsgWidget*>(host.render().widget());
}

} // namespace cloudsim::host
