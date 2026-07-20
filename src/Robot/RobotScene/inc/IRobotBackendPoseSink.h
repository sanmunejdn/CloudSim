#ifndef ROBOTSCENE_IROBOTBACKENDPOSESINK_H
#define ROBOTSCENE_IROBOTBACKENDPOSESINK_H

/// @file IRobotBackendPoseSink.h
/// @brief OSG 后端根位姿读写（Widget OsgWidget 实现）

#include "robot_scene_global.h"

#include <string>

#include <osg/Matrixd>

class BackendDataBase;

/// OSG 后端根位姿读写（Widget OsgWidget 实现）
class ROBOT_SCENE_API IRobotBackendPoseSink
{
public:
	virtual ~IRobotBackendPoseSink() = default;

	virtual bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const = 0;
	virtual void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) = 0;

	/// per-link URDF：mesh 模型中心 mm，配合 backend_pose_euler_from_world_mat
	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
											double& outCz) const
	{
		(void)backendId;
		(void)outCx;
		(void)outCy;
		(void)outCz;
		return false;
	}

	/// FK 写 pose/rotation 后同步 OSG 外支 PAT
	virtual void syncRobotMeshBackendPoseAfterKinematics(const BackendDataBase& mesh) { (void)mesh; }
};

#endif // ROBOTSCENE_IROBOTBACKENDPOSESINK_H
