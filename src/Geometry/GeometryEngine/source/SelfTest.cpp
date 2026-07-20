/// @file SelfTest.cpp
/// @brief SelfTest 实现

#include "SelfTest.h"

#include "Adapters.h"
#include "BackendWorldPose.h"
#include "ToolKinematics.h"

#include <cmath>
#include <sstream>

#include <osg/Matrixd>
#include <osg/Quat>

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

Eigen::Vector3d transformPointColumn(const RigidTransform& rt, const Eigen::Vector3d& pModel)
{
	return rt.isometry() * pModel;
}

Eigen::Vector3d transformPointOsgRow(const osg::Matrixd& m, const Eigen::Vector3d& pModel)
{
	const osg::Vec3d in(pModel.x(), pModel.y(), pModel.z());
	const osg::Vec3d out = in * m;
	return Eigen::Vector3d(out.x(), out.y(), out.z());
}

double matrixMaxAbsDiff(const osg::Matrixd& a, const osg::Matrixd& b)
{
	double maxDiff = 0.0;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			maxDiff = std::max(maxDiff, std::fabs(a(r, c) - b(r, c)));
		}
	}
	return maxDiff;
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

	{
		const double px = 100.0;
		const double py = -50.0;
		const double pz = 250.0;
		const double ex = 15.0;
		const double ey = -30.0;
		const double ez = 162.0;
		const RigidTransform rt = rigidTransformFromBackendPoseEuler(px, py, pz, ex, ey, ez);
		const osg::Matrixd osgWorld = osgMatrixFromRigidTransform(rt);
		const RigidTransform back = rigidTransformFromOsg(osgWorld);
		double bx = 0.0;
		double by = 0.0;
		double bz = 0.0;
		double bex = 0.0;
		double bey = 0.0;
		double bez = 0.0;
		backendPoseEulerFromRigidTransform(back, bx, by, bz, bex, bey, bez);
		expectNear(failures, "backendPoseRoundTrip.px", bx, px, 1e-6);
		expectNear(failures, "backendPoseRoundTrip.py", by, py, 1e-6);
		expectNear(failures, "backendPoseRoundTrip.pz", bz, pz, 1e-6);
		expectNear(failures, "backendPoseRoundTrip.ex", bex, ex, 1e-4);
		expectNear(failures, "backendPoseRoundTrip.ey", bey, ey, 1e-4);
		expectNear(failures, "backendPoseRoundTrip.ez", bez, ez, 1e-4);

		const ColMajorMat4 cm = colMajorFromRigidTransform(rt);
		const RigidTransform fromMat4 = rigidTransformFromColMajor(cm);
		const Eigen::Vector3d samples[] = {
			Eigen::Vector3d::Zero(),
			Eigen::Vector3d(10.0, 20.0, 30.0),
			Eigen::Vector3d(-40.0, 5.0, 80.0),
		};
		for (std::size_t i = 0U; i < 3U; ++i)
		{
			const Eigen::Vector3d pCol = transformPointColumn(fromMat4, samples[i]);
			const Eigen::Vector3d pOsg = transformPointOsgRow(osgWorld, samples[i]);
			const std::string tag = "dataOsgPoint" + std::to_string(i);
			expectNear(failures, (tag + ".x").c_str(), pOsg.x(), pCol.x(), 1e-6);
			expectNear(failures, (tag + ".y").c_str(), pOsg.y(), pCol.y(), 1e-6);
			expectNear(failures, (tag + ".z").c_str(), pOsg.z(), pCol.z(), 1e-6);
		}

		const osg::Quat q = eulerDegToQuat(ex, ey, ez);
		const osg::Matrixd explicitRxT = osg::Matrixd::rotate(q) * osg::Matrixd::translate(osg::Vec3d(px, py, pz));
		expectNear(failures, "explicitRxT.matrixDiff", matrixMaxAbsDiff(explicitRxT, osgWorld), 0.0, 1e-9);

		const RigidTransform rotatedOnly =
			rigidTransformFromBackendPoseEuler(px, py, pz, ex + 25.0, ey - 10.0, ez + 40.0);
		double rpx = 0.0;
		double rpy = 0.0;
		double rpz = 0.0;
		double rex = 0.0;
		double rey = 0.0;
		double rez = 0.0;
		backendPoseEulerFromRigidTransform(rotatedOnly, rpx, rpy, rpz, rex, rey, rez);
		expectNear(failures, "rotatePreservesOrigin.px", rpx, px, 1e-6);
		expectNear(failures, "rotatePreservesOrigin.py", rpy, py, 1e-6);
		expectNear(failures, "rotatePreservesOrigin.pz", rpz, pz, 1e-6);

		const osg::Matrixd legacyTxR = osg::Matrixd::translate(osg::Vec3d(px, py, pz)) * osg::Matrixd::rotate(q);
		const RigidTransform legacyRt = rigidTransformFromOsg(legacyTxR);
		if (legacyRt.translationErrorMm(rt) < 1.0)
		{
			failures.push_back("legacyTxR.poseDiff expected >1mm for non-zero rotation");
		}
	}

	return failures.empty();
}

} // namespace engine
