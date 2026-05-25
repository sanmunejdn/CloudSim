#pragma once

#include "cloudsim_host_global.h"

#include <string>

namespace cloudsim::host {

class DocumentHost;

/// 写 hierarchy Follow
CLOUDSIM_HOST_EXPORT void applyHierarchyFollowBinding(DocumentHost& host, const std::string& childBackendId,
	const std::string& parentBackendId);

} // namespace cloudsim::host
