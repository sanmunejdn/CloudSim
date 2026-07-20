#ifndef CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H
#define CLOUDSIMHOST_BACKENDHIERARCHYFOLLOW_H

/// @file BackendHierarchyFollow.h
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
