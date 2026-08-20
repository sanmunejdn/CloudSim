#ifndef CLOUDSIMHOST_PERLINKKINEMATICSHOSTIMPL_H
#define CLOUDSIMHOST_PERLINKKINEMATICSHOSTIMPL_H

/// @file PerLinkKinematicsHostImpl.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief per-link 机器人运动学宿主实现（Host 编译单元，可安全依赖 RobotSceneKinematics / UrdfRobotLoader）

#include "cloudsim_host_global.h"

#include "IPerLinkKinematicsHost.h"
#include "IPerLinkRobotStateAccessor.h"

namespace cloudsim::host
{
/// per-link 机器人运动学宿主实现（Host 编译单元，可安全依赖 RobotSceneKinematics / UrdfRobotLoader）
class CLOUDSIM_HOST_EXPORT PerLinkKinematicsHostImpl : public IPerLinkKinematicsHost
{
public:
	explicit PerLinkKinematicsHostImpl(IPerLinkRobotStateAccessor* accessor);

	bool applyPerLinkRobotFkFromGizmoAnchor(int instanceIndex, const QString& anchorLinkBackendId,
											const QVector<double>& jointAnglesRad) override;

	void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) override;

private:
	IPerLinkRobotStateAccessor* m_accessor = nullptr;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_PERLINKKINEMATICSHOSTIMPL_H
