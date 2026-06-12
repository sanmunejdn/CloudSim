#pragma once

#include "backendvisual_global.h"

#include "BackendDataBase.h"

#include <osg/Matrixd>

namespace backend_pose_osg
{

/// backend.pose/rotation ↔ OSG 世界刚体矩阵（pose=模型原点世界坐标；权威 engine::BackendWorldPose）
BACKENDVISUAL_EXPORT osg::Matrixd worldMatrixFromBackendPoseEuler(
	const BackendVec3& pose,
	const BackendVec3& eulerDeg);

BACKENDVISUAL_EXPORT void backendPoseEulerFromWorldMatrix(
	const osg::Matrixd& world,
	BackendVec3& outPose,
	BackendVec3& outEulerDeg);

} // namespace backend_pose_osg
