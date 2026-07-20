#ifndef GEOMETRYENGINE_BACKENDWORLDPOSE_H
#define GEOMETRYENGINE_BACKENDWORLDPOSE_H

/// @file BackendWorldPose.h
/// @brief pose = 模型坐标原点在世界中的位置 (mm)；euler = 内禀 ZYX 度

#include "geometry_engine_global.h"

#include "RigidTransform.h"

namespace engine
{
/// pose = 模型坐标原点在世界中的位置 (mm)；euler = 内禀 ZYX 度
/// p_world = R(rotation) * p_model + pose（列向量）/ p_model × R + pose（OSG 行向量经 Adapters 桥接）
GEOMETRY_ENGINE_API RigidTransform rigidTransformFromBackendPoseEuler(double pxMm, double pyMm, double pzMm,
																	  double exDeg, double eyDeg, double ezDeg);

GEOMETRY_ENGINE_API void backendPoseEulerFromRigidTransform(const RigidTransform& rt, double& outPxMm, double& outPyMm,
															double& outPzMm, double& outExDeg, double& outEyDeg,
															double& outEzDeg);

} // namespace engine

#endif // GEOMETRYENGINE_BACKENDWORLDPOSE_H
