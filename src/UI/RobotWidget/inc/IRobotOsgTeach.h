#ifndef ROBOTWIDGET_IROBOTOSGTEACH_H
#define ROBOTWIDGET_IROBOTOSGTEACH_H

/// @file IRobotOsgTeach.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人 UI TCP 拖动示教（IRobotOsgViewHost 切面）

#include "robotwidget_global.h"

#include "CoreTypes.h"

#include <functional>
#include <string>

namespace engine
{
class RigidTransform;
}

/// TCP 拖动示教与目标位姿同步
class ROBOTWIDGET_EXPORT IRobotOsgTeach
{
public:
	virtual ~IRobotOsgTeach() = default;

	virtual bool isTcpDragTeachActive() const = 0;
	virtual void endTcpDragTeach() = 0;
	virtual void beginTcpDragTeach(const std::string& mountBackendId, const engine::RigidTransform& T_base_target,
								   float modelDiagonalMm,
								   std::function<bool(cloudsim::core::Mat4& outRobotBaseWorld)> resolveRobotBaseWorld,
								   const cloudsim::core::Mat4* toolLocalOnFlange) = 0;
	virtual void updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target,
											  bool syncTargetInBase = true) = 0;
	virtual void updateTcpDragTeachToolLocalOnFlange(const cloudsim::core::Mat4& toolLocalOnFlange) = 0;
	virtual engine::RigidTransform tcpDragTeachTargetInBase() const = 0;
};

#endif // ROBOTWIDGET_IROBOTOSGTEACH_H
