#ifndef CLOUDSIMHOST_DOCUMENTHOSTACCESS_H
#define CLOUDSIMHOST_DOCUMENTHOSTACCESS_H

/// @file DocumentHostAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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

/// Host 内部取 backend 管理器的唯一收口：仅用于把管理器传给引擎/库函数；
/// 对象级操作优先用 DocumentHost 窄接口（findObject/backendContains 等），勿在 UI 层扩散
inline BackendDataManager& backendManagerOf(DocumentHost& host)
{
	return host.backend();
}

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_DOCUMENTHOSTACCESS_H
