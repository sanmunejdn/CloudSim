#ifndef CLOUDSIMHOST_IPERLINKKINEMATICSHOST_H
#define CLOUDSIMHOST_IPERLINKKINEMATICSHOST_H

/// @file IPerLinkKinematicsHost.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief per-link 机器人运动学计算宿主接口（由 Host 实现，DocumentPage 通过接口调用）

#include "cloudsim_host_global.h"

#include <QString>
#include <QVector>

namespace cloudsim::host
{
/// per-link 机器人运动学计算宿主接口（由 Host 实现，DocumentPage 通过接口调用）
/// 目的：让 DocumentPage 不直接依赖 RobotSceneKinematics / UrdfRobotLoader
class CLOUDSIM_HOST_EXPORT IPerLinkKinematicsHost
{
public:
	virtual ~IPerLinkKinematicsHost() = default;

	/// per-link 机器人：gizmo 锚点反解 basePlacement 并 FK
	virtual bool applyPerLinkRobotFkFromGizmoAnchor(int instanceIndex, const QString& anchorLinkBackendId,
													const QVector<double>& jointAnglesRad) = 0;

	/// per-link 机器人：根据当前关节角重算 outer bind
	virtual void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) = 0;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_IPERLINKKINEMATICSHOST_H
