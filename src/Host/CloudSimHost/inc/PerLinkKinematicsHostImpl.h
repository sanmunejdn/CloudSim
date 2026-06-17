#pragma once

#include "cloudsim_host_global.h"
#include "IPerLinkKinematicsHost.h"
#include "IPerLinkRobotStateAccessor.h"

namespace cloudsim::host {

/// per-link 机器人运动学宿主实现（Host 编译单元，可安全依赖 RobotSceneKinematics / UrdfRobotLoader）
class CLOUDSIM_HOST_EXPORT PerLinkKinematicsHostImpl : public IPerLinkKinematicsHost
{
public:
	explicit PerLinkKinematicsHostImpl(IPerLinkRobotStateAccessor* accessor);

	bool applyPerLinkRobotFkFromGizmoAnchor(
		int instanceIndex,
		const QString& anchorLinkBackendId,
		const QVector<double>& jointAnglesRad) override;

	void reconcilePerLinkOuterBindFromScene(
		int instanceIndex,
		const QVector<double>& jointAnglesRad) override;

private:
	IPerLinkRobotStateAccessor* m_accessor = nullptr;
};

} // namespace cloudsim::host