#ifndef ROBOTSCENE_IROBOTBACKENDPOSESINK_H
#define ROBOTSCENE_IROBOTBACKENDPOSESINK_H

/// @file IRobotBackendPoseSink.h
/// @brief 后端根位姿读写（无 osg；矩阵为列主序 Mat4）

#include "robot_scene_global.h"

#include "CoreTypes.h"

#include <string>

class BackendDataBase;

/// 后端根位姿读写（OsgWidget 实现；OSG 转换留在实现内）
class ROBOT_SCENE_API IRobotBackendPoseSink
{
public:
	virtual ~IRobotBackendPoseSink() = default;

	virtual bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const = 0;
	virtual void setBackendRootWorldMatrixFromWorld(const std::string& backendId,
													const cloudsim::core::Mat4& worldColumnMajor) = 0;

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
