#pragma once

#include "cloudsim_host_global.h"

#include <functional>
#include <string>

class OsgWidget;

namespace cloudsim::host {

class DocumentHost;

/// 交互守卫由 Widget 注入，Host 不依赖 RobotSimulationController
struct FollowSolveContext {
	std::function<bool()> skipAll;
	std::function<bool(std::string& outSelectedId)> fillGizmoSelectedId;
};

/// Follow 求解 + 脏集写回 OSG；守卫策略留在 Widget
CLOUDSIM_HOST_EXPORT void runBackendFollowSolveAndSync(DocumentHost& page, OsgWidget& osg,
	const FollowSolveContext* ctx = nullptr, const std::string* manualPoseAuthorityBackendId = nullptr);

} // namespace cloudsim::host
