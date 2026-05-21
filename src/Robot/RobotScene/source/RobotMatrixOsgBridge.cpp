#include "RobotMatrixOsgBridge.h"
#include "RobotCoordinateFrames.h"

#include <Adapters.h>
#include <SelfTest.h>
#include <ToolKinematics.h>

#include <cmath>
#include <sstream>

namespace RobotMatrixOsg
{

osg::Matrixd matrixFromBackendColMajor(const BackendMat4& m)
{
	return engine::osgMatrixFromRigidTransform(RobotCoordinate::rigidTransformFromBackendMat4(m));
}

BackendMat4 backendColMajorFromMatrix(const osg::Matrixd& m)
{
	return RobotCoordinate::backendMat4FromRigidTransform(engine::rigidTransformFromOsg(m));
}

BackendMat4 targetInBaseFromFlangeLinkWorld(
	const osg::Matrixd& T_base_flange_osg,
	const BackendMat4& T_flange_tool)
{
	const engine::RigidTransform flange = engine::rigidTransformFromOsg(T_base_flange_osg);
	const engine::RigidTransform tool = RobotCoordinate::rigidTransformFromBackendMat4(T_flange_tool);
	return RobotCoordinate::backendMat4FromRigidTransform(engine::toolOriginFromFlange(flange, tool));
}

BackendMat4 flangeTargetFromToolOriginInBase(
	const BackendMat4& T_base_target,
	const BackendMat4& T_flange_tool)
{
	return RobotCoordinate::flangeTargetFromToolOriginInBase(T_base_target, T_flange_tool);
}

BackendMat4 targetInBaseFromFlange(const BackendMat4& T_base_flange, const BackendMat4& T_flange_tool)
{
	return RobotCoordinate::targetInBaseFromFlange(T_base_flange, T_flange_tool);
}

osg::Matrixd matrixOsgFromPoseMmDeg(
	double px,
	double py,
	double pz,
	double exDeg,
	double eyDeg,
	double ezDeg)
{
	const engine::RigidTransform t = engine::RigidTransform::fromTranslationEulerDeg(px, py, pz, exDeg, eyDeg, ezDeg);
	return engine::osgMatrixFromRigidTransform(t);
}

namespace
{

void expectNear(std::vector<std::string>& failures, const char* name, double actual, double expected, double eps)
{
	if (!std::isfinite(actual) || std::fabs(actual - expected) > eps)
	{
		std::ostringstream oss;
		oss << name << " expected " << expected << " got " << actual;
		failures.push_back(oss.str());
	}
}

} // namespace

bool runConventionSelfTest(std::vector<std::string>& failures)
{
	failures.clear();
	if (!engine::runSelfTest(failures))
	{
		return false;
	}

	const BackendMat4 T_target = RobotCoordinate::targetInBaseFromPose(500.0, 600.0, 700.0, 5.0, -10.0, 15.0);
	const BackendMat4 T_tool = BackendMat4::translate(200.0, 0.0, -50.0);
	const BackendMat4 T_flange = flangeTargetFromToolOriginInBase(T_target, T_tool);
	const BackendMat4 T_back = targetInBaseFromFlange(T_flange, T_tool);
	const RobotCoordinate::RobotRigidFrame f0 = RobotCoordinate::mat4ToFrame(T_target);
	const RobotCoordinate::RobotRigidFrame f1 = RobotCoordinate::mat4ToFrame(T_back);
	expectNear(failures, "bridgeRoundTrip.x", f1.positionMm[0], f0.positionMm[0], 1e-3);
	expectNear(failures, "bridgeRoundTrip.y", f1.positionMm[1], f0.positionMm[1], 1e-3);
	expectNear(failures, "bridgeRoundTrip.z", f1.positionMm[2], f0.positionMm[2], 1e-3);

	return failures.empty();
}

} // namespace RobotMatrixOsg
