#pragma once

#include "robot_scene_global.h"

#include <osg/Matrixd>

#include <string>

class BackendDataBase;

/// OSG backend root pose get/set (implemented by \ref OsgWidget in Widget).
class ROBOT_SCENE_API IRobotBackendPoseSink
{
public:
	virtual ~IRobotBackendPoseSink() = default;

	virtual bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const = 0;
	virtual void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) = 0;

	/// Optional: mesh model center (mm) used with \ref backend_pose_euler_from_world_mat for per-link URDF drivers.
	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy, double& outCz) const
	{
		(void)backendId;
		(void)outCx;
		(void)outCy;
		(void)outCz;
		return false;
	}

	/// Sync OSG outer PAT from mesh backend pose/rotation after kinematics writes.
	virtual void syncRobotMeshBackendPoseAfterKinematics(const BackendDataBase& mesh) { (void)mesh; }
};
