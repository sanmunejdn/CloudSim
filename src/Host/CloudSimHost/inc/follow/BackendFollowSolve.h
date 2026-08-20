#ifndef CLOUDSIMHOST_BACKENDFOLLOWSOLVE_H
#define CLOUDSIMHOST_BACKENDFOLLOWSOLVE_H

/// @file BackendFollowSolve.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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

/// Follow 求解；osg 可空（Web Headless 只写 BackendData，靠 worldMatrix 回落）
CLOUDSIM_HOST_EXPORT void runBackendFollowSolveAndSync(DocumentHost& page, OsgWidget* osg,
													   const FollowSolveContext* ctx = nullptr,
													   const std::string* manualPoseAuthorityBackendId = nullptr);

/// follow.* 属性提交后重算局部偏移并置脏
CLOUDSIM_HOST_EXPORT void afterFollowPropertyEdited(DocumentHost& host, const QString& backendId,
													const QString& propertyKey, const QString& valueText);

/// 手动改 follower 位姿后烘焙 local，避免松手被旧偏移拽回
CLOUDSIM_HOST_EXPORT void bakeFollowLocalAfterManualPoseEdit(DocumentHost& host, const std::string& backendId);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDFOLLOWSOLVE_H
