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
	const engine::RigidTransform rt = engine::rigidTransformFromBackendPoseEuler(
		pose.x,
		pose.y,
		pose.z,
		eulerDeg.x,
		eulerDeg.y,
		eulerDeg.z);
	return engine::osgMatrixFromRigidTransform(rt);
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
