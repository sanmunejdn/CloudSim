#include "SelfTest.h"

#include "Adapters.h"
#include "ToolKinematics.h"

#include <osg/Matrixd>

#include <cmath>
#include <sstream>

namespace engine
{

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

RigidTransform translationOnly(double x, double y, double z)
{
	return RigidTransform::fromTranslationQuat(Eigen::Vector3d(x, y, z), Eigen::Quaterniond::Identity());
}

} // namespace

bool runSelfTest(std::vector<std::string>& failures)
{
	failures.clear();

	{
		const RigidTransform flange = translationOnly(1055.0, 0.0, 1055.0);
		const RigidTransform tool = translationOnly(200.0, 0.0, 0.0);
		const RigidTransform target = toolOriginFromFlange(flange, tool);
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		target.translationMm(x, y, z);
		expectNear(failures, "urdfToolCompose.x", x, 1255.0, 1e-3);
		expectNear(failures, "urdfToolCompose.z", z, 1055.0, 1e-3);

		const RigidTransform flangeBack = flangeFromToolOrigin(target, tool);
		double fx = 0.0;
		double fy = 0.0;
		double fz = 0.0;
		flangeBack.translationMm(fx, fy, fz);
		expectNear(failures, "urdfToolInverse.x", fx, 1055.0, 1e-3);
		expectNear(failures, "urdfToolInverse.z", fz, 1055.0, 1e-3);
	}

	{
		const RigidTransform a = RigidTransform::fromTranslationEulerDeg(500.0, 600.0, 700.0, 5.0, -10.0, 15.0);
		const RigidTransform tool = translationOnly(200.0, 0.0, -50.0);
		const RigidTransform flange = flangeFromToolOrigin(a, tool);
		const RigidTransform back = toolOriginFromFlange(flange, tool);
		expectNear(failures, "roundTrip.pos", back.translationErrorMm(a), 0.0, 1e-3);
		expectNear(failures, "roundTrip.rot", back.rotationErrorDeg(a), 0.0, 1e-2);
	}

	{
		const RigidTransform t = RigidTransform::fromTranslationEulerDeg(100.0, 200.0, 300.0, 15.0, -30.0, 45.0);
		const osg::Matrixd o = osgMatrixFromRigidTransform(t);
		const RigidTransform back = rigidTransformFromOsg(o);
		expectNear(failures, "osgRoundTrip.pos", back.translationErrorMm(t), 0.0, 1e-6);
		expectNear(failures, "osgRoundTrip.rot", back.rotationErrorDeg(t), 0.0, 1e-4);
	}

	{
		const RigidTransform t = RigidTransform::fromTranslationEulerDeg(500.0, 600.0, 700.0, 5.0, -10.0, 15.0);
		const ColMajorMat4 cm = colMajorFromRigidTransform(t);
		const RigidTransform back = rigidTransformFromColMajor(cm);
		expectNear(failures, "colMajorRoundTrip.pos", back.translationErrorMm(t), 0.0, 1e-6);
		expectNear(failures, "colMajorRoundTrip.rot", back.rotationErrorDeg(t), 0.0, 1e-4);
	}

	// 统一到工具学口径，避免行/列链路混用导致自检误判
	{
		const RigidTransform flange = RigidTransform::fromTranslationEulerDeg(1000.0, 500.0, 800.0, 0.0, 90.0, 0.0);
		const RigidTransform tool = translationOnly(0.0, 0.0, -200.0);
		const RigidTransform tcp = toolOriginFromFlange(flange, tool);
		double tx = 0.0;
		double ty = 0.0;
		double tz = 0.0;
		tcp.translationMm(tx, ty, tz);
		const RigidTransform tcpByToolKinematics = toolOriginFromFlange(flange, tool);
		expectNear(failures, "flangeLocalToolOffset.deltaX", tx - 1000.0, 200.0, 1e-3);
		expectNear(failures, "flangeLocalToolOffset.deltaY", ty - 500.0, 0.0, 1e-3);
		expectNear(failures, "flangeLocalToolOffset.deltaZ", tz - 800.0, 0.0, 1e-3);
		expectNear(failures, "flangeLocalToolOffset.refPos", tcpByToolKinematics.translationErrorMm(tcp), 0.0, 1e-3);
	}

	return failures.empty();
}

} // namespace engine
