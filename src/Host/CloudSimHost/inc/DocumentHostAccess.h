#ifndef CLOUDSIMHOST_DOCUMENTHOSTACCESS_H
#define CLOUDSIMHOST_DOCUMENTHOSTACCESS_H

/// @file DocumentHostAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Host 内部取 BackendDataManager 的唯一收口

#include "DocumentHost.h"

namespace cloudsim::host
{
/// 仅用于把管理器传给引擎/库函数；对象级操作优先 DocumentHost 窄接口
inline BackendDataManager& backendManagerOf(DocumentHost& host)
{
	return host.backend();
}

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_DOCUMENTHOSTACCESS_H
