#ifndef CLOUDSIMHOST_BACKENDFOLLOWSOLVE_H
#define CLOUDSIMHOST_BACKENDFOLLOWSOLVE_H

/// @file BackendFollowSolve.h
/// @brief Follow 求解上下文

#include "cloudsim_host_global.h"

#include <functional>
#include <string>

class OsgWidget;

namespace cloudsim::host
{
class DocumentHost;

/// Follow 求解上下文
struct FollowSolveContext
{
	std::function<bool()> skipAll;
	std::function<bool(std::string& outSelectedId)> fillGizmoSelectedId;
};

/// Follow 求解写 OSG
CLOUDSIM_HOST_EXPORT void runBackendFollowSolveAndSync(DocumentHost& page, OsgWidget& osg,
													   const FollowSolveContext* ctx = nullptr,
													   const std::string* manualPoseAuthorityBackendId = nullptr);

/// follow.* 属性提交后重算局部偏移并置脏
CLOUDSIM_HOST_EXPORT void afterFollowPropertyEdited(DocumentHost& host, const QString& backendId,
													const QString& propertyKey, const QString& valueText);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDFOLLOWSOLVE_H
