#pragma once

#include "cloudsim_host_global.h"

#include <string>

namespace cloudsim::host {

class DocumentHost;

/// 工程/树父子边：写 hierarchyDriven Follow；求解由 BackendFollowSolve 统一触发
CLOUDSIM_HOST_EXPORT void applyHierarchyFollowBinding(DocumentHost& host, const std::string& childBackendId,
	const std::string& parentBackendId);

} // namespace cloudsim::host
