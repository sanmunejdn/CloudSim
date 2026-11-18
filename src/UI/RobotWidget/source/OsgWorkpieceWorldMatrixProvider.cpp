/// @file OsgWorkpieceWorldMatrixProvider.cpp
/// @brief 桌面工件世界矩阵与 OSG→Backend 同步

#include "OsgWorkpieceWorldMatrixProvider.h"

#include "BackendDataBase.h"
#include "BackendFollowMath.h"
#include "IRobotOsgSceneOps.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotSimulationMath.h"

#include <CoreTypes.h>

bool OsgWorkpieceWorldMatrixProvider::getWorldMatrix(const std::string& backendId, BackendMat4& out) const
{
	if (!m_osg || backendId.empty())
	{
		return false;
	}
	// 与 FeaturePickTransform 一致：B-rep 视觉根可能是 alias，不能用逻辑 id 直查
	const std::string xformId = m_osg->resolvePickScopeBackendId(backendId);
	cloudsim::core::Mat4 packed{};
	if (!m_osg->getBackendRootWorldMatrix(xformId, packed))
	{
		if (xformId == backendId || !m_osg->getBackendRootWorldMatrix(backendId, packed))
		{
			return false;
		}
	}
	out = RobotMatrixOsg::backendColMajorFromMatrix(RobotSimulationMath::osgMatrixFromCoreMat4(packed));
	return true;
}

bool syncBackendWorldPoseFromOsg(IRobotOsgSceneOps* osg, BackendDataBase& data)
{
	if (!osg || !data.hasPoseProperty())
	{
		return false;
	}
	const std::string xformId = osg->resolvePickScopeBackendId(data.id());
	cloudsim::core::Mat4 packed{};
	if (!osg->getBackendRootWorldMatrix(xformId, packed) &&
		(xformId == data.id() || !osg->getBackendRootWorldMatrix(data.id(), packed)))
	{
		return false;
	}
	const BackendMat4 world =
		RobotMatrixOsg::backendColMajorFromMatrix(RobotSimulationMath::osgMatrixFromCoreMat4(packed));
	BackendVec3 poseB{};
	BackendVec3 eulerB{};
	backend_pose_euler_from_world_mat(world, poseB, eulerB);
	if (data.supportsBackendTransform())
	{
		data.applyBackendWorldPose(poseB, eulerB);
	}
	else
	{
		data.setPose(poseB);
		if (data.hasRotationProperty())
		{
			data.setRotation(eulerB);
		}
	}
	return true;
}
