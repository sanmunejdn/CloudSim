#ifndef CLOUDSIMHOST_DOCUMENTHOSTACCESS_H
#define CLOUDSIMHOST_DOCUMENTHOSTACCESS_H

/// @file DocumentHostAccess.h
/// @brief Host 取 OsgWidget（直接读成员，避免构造期经 render() 递归）

#include "DocumentHost.h"
#include "OsgWidget.h"

namespace cloudsim::host
{
/// Host 取 OsgWidget（直接读成员，避免构造期经 render() 递归）
inline OsgWidget* osgWidgetFrom(DocumentHost& host)
{
	return host.osgWidget();
}

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_DOCUMENTHOSTACCESS_H
