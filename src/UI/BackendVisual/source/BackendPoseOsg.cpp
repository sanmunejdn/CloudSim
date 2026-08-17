/// @file BackendPoseOsg.cpp
/// @brief 后端姿态 OSG 适配

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendPoseOsg.h"

#include <Adapters.h>
#include <BackendWorldPose.h>
#include <osg/Matrixd>

namespace backend_pose_osg
{
osg::Matrixd worldMatrixFromBackendPoseEuler(const BackendVec3& pose, const BackendVec3& eulerDeg)
{
	const engine::RigidTransform rt =
		engine::rigidTransformFromBackendPoseEuler(pose.x, pose.y, pose.z, eulerDeg.x, eulerDeg.y, eulerDeg.z);
	return engine::osgMatrixFromRigidTransform(rt);
}

osg::Matrixd osgMatrixFromBackendWorldMatrix(const BackendMat4& world)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = world.v[i];
	}
	return engine::osgMatrixFromRigidTransform(engine::rigidTransformFromColMajor(cm));
}

BackendMat4 backendWorldMatrixFromOsgMatrix(const osg::Matrixd& world)
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(engine::rigidTransformFromOsg(world));
	BackendMat4 out{};
	for (int i = 0; i < 16; ++i)
	{
		out.v[i] = cm[static_cast<size_t>(i)];
	}
	return out;
}

void backendPoseEulerFromWorldMatrix(const osg::Matrixd& world, BackendVec3& outPose, BackendVec3& outEulerDeg)
{
	const engine::RigidTransform rt = engine::rigidTransformFromOsg(world);
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	engine::backendPoseEulerFromRigidTransform(rt, px, py, pz, ex, ey, ez);
	outPose.x = px;
	outPose.y = py;
	outPose.z = pz;
	outEulerDeg.x = ex;
	outEulerDeg.y = ey;
	outEulerDeg.z = ez;
}

} // namespace backend_pose_osg
