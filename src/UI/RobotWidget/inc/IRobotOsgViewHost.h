#pragma once

#include "RobotOsgUiTypes.h"
#include "robotwidget_global.h"

#include <functional>
#include <string>
#include <vector>

#include <osg/Matrixd>

class IRobotBackendPoseSink;
namespace engine { class RigidTransform; }

/// OSG / 3D view operations for robot UI (implemented by Widget \ref OsgWidget wrapper).
class ROBOTWIDGET_EXPORT IRobotOsgViewHost
{
public:
	virtual ~IRobotOsgViewHost() = default;

	virtual IRobotBackendPoseSink* poseSink() = 0;
	virtual void requestRedraw() = 0;

	virtual bool objectSelectionMode() const = 0;
	virtual void setObjectSelectionMode(bool enabled) = 0;
	virtual void clearBackendObjectSelection() = 0;
	virtual void setSelectionActive(bool active) = 0;
	virtual void setTransformGizmoFrame(int worldOrLocal) = 0;

	virtual bool hasBackendObjectBranch(const std::string& backendId) const = 0;
	virtual bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const = 0;
	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy, double& cz) const = 0;

	virtual void setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes) = 0;
	virtual void clearInstructionPoseAxes() = 0;
	virtual void setCameraFollowBackendId(const std::string& backendId) = 0;

	virtual void setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update) = 0;
	virtual void clearRobotFrameOverlays(const std::string& robotRootBackendId) = 0;

	virtual bool isTcpDragTeachActive() const = 0;
	virtual void endTcpDragTeach() = 0;
	virtual void beginTcpDragTeach(
		const std::string& mountBackendId,
		const engine::RigidTransform& T_base_target,
		float modelDiagonalMm,
		std::function<bool(osg::Matrixd& outRobotBaseWorld)> resolveRobotBaseWorld,
		const osg::Matrixd* toolLocalOnFlange) = 0;
	virtual void updateTcpDragTeachFromTarget(
		const engine::RigidTransform& T_base_target,
		bool syncTargetInBase = true) = 0;
	virtual void updateTcpDragTeachToolLocalOnFlange(const osg::Matrixd& toolLocalOnFlange) = 0;
};
