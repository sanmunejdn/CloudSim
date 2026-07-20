#ifndef BACKENDVISUAL_BACKENDPOSEOSG_H
#define BACKENDVISUAL_BACKENDPOSEOSG_H

/// @file BackendPoseOsg.h
/// @brief backend.pose/rotation ↔ OSG 世界刚体矩阵（pose=模型原点世界坐标；权威 engine::BackendWorldPose）

#include "backendvisual_global.h"

#include "BackendDataBase.h"

#include <osg/Matrixd>

namespace backend_pose_osg
{
/// backend.pose/rotation ↔ OSG 世界刚体矩阵（pose=模型原点世界坐标；权威 engine::BackendWorldPose）
BACKENDVISUAL_EXPORT osg::Matrixd worldMatrixFromBackendPoseEuler(const BackendVec3& pose, const BackendVec3& eulerDeg);

/// 权威 BackendMat4 → OSG 世界矩阵（显示/syncOuterPat 须走 worldMatrix，勿再分解 pose/rotation）
BACKENDVISUAL_EXPORT osg::Matrixd osgMatrixFromBackendWorldMatrix(const BackendMat4& world);

BACKENDVISUAL_EXPORT BackendMat4 backendWorldMatrixFromOsgMatrix(const osg::Matrixd& world);

BACKENDVISUAL_EXPORT void backendPoseEulerFromWorldMatrix(const osg::Matrixd& world, BackendVec3& outPose,
														  BackendVec3& outEulerDeg);

} // namespace backend_pose_osg

#endif // BACKENDVISUAL_BACKENDPOSEOSG_H
