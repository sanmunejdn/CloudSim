#ifndef CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H
#define CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H

/// @file BackendHierarchyFollow.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 写 hierarchy Follow

#include "cloudsim_host_global.h"

#include <string>

namespace cloudsim::host
{
class DocumentHost;

/// 写 hierarchy Follow
CLOUDSIM_HOST_EXPORT void applyHierarchyFollowBinding(DocumentHost& host, const std::string& childBackendId,
													  const std::string& parentBackendId);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H
