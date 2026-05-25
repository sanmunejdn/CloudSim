#pragma once

#include "DocumentHost.h"
#include "IRenderView.h"
#include "OsgWidget.h"

namespace cloudsim::host {

/// Host 取 OsgWidget
inline OsgWidget* osgWidgetFrom(DocumentHost& host)
{
	return qobject_cast<OsgWidget*>(host.render().widget());
}

} // namespace cloudsim::host
