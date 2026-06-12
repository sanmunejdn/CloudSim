#include "BackendWorldPose.h"

namespace engine
{

RigidTransform rigidTransformFromBackendPoseEuler(
	const double pxMm,
	const double pyMm,
	const double pzMm,
	const double exDeg,
	const double eyDeg,
	const double ezDeg)
{
	return RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
}

void backendPoseEulerFromRigidTransform(
	const RigidTransform& rt,
	double& outPxMm,
	double& outPyMm,
	double& outPzMm,
	double& outExDeg,
	double& outEyDeg,
	double& outEzDeg)
{
	rt.translationMm(outPxMm, outPyMm, outPzMm);
	rt.eulerDegForDisplay(outExDeg, outEyDeg, outEzDeg);
}

} // namespace engine
