#ifndef CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H
#define CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H

/// @file BackendHierarchyFollow.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 显式写 hierarchy Follow（勿作 attach/edges 默认行为；同部件用 compound）

#include "cloudsim_host_global.h"

#include <string>

namespace cloudsim::host
{
class DocumentHost;

/// 显式层级 Follow；attach/edges 默认路径不得调用
CLOUDSIM_HOST_EXPORT void applyHierarchyFollowBinding(DocumentHost& host, const std::string& childBackendId,
													  const std::string& parentBackendId);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H
