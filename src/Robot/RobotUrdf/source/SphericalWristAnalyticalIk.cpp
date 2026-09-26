/// @file SphericalWristAnalyticalIk.cpp
/// @brief 球形腕 6R：腕心交汇检测 + 臂位置数值 + 腕 ZYZ 解析多解

#include "SphericalWristAnalyticalIk.h"

#include "IkSeedExpand.h"
#include "KinematicCoreUrdfIk.h"
#include "KinematicGraph.h"
#include "TreeForwardKinematics.h"
#include "UrdfRobotLoader.h"

#include <QVector>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include <osg/Matrixd>
#include <osg/Quat>

namespace UrdfRobotLoader
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kWristConcurMm = 5.0;

void identity16(double m[16])
{
	std::memset(m, 0, 16 * sizeof(double));
	m[0] = m[5] = m[10] = m[15] = 1.0;
}

bool collectRevoluteJoints(const kinematic_core::KinematicGraph& graph, std::vector<const kinematic_core::KinematicJoint*>& out)
{
	out.clear();
	for (const auto& j : graph.joints)
	{
		if (!j.motion.enabled || j.qIndex < 0)
		{
			continue;
		}
		if (j.motion.motionType != kinematic_core::JointMotionType::Revolute)
		{
			return false;
		}
		out.push_back(&j);
	}
	return static_cast<int>(out.size()) == graph.dofCount();
}

bool axisLineFromJoint(const kinematic_core::KinematicGraph& graph, const kinematic_core::KinematicJoint& j,
					   const std::vector<std::array<double, 16>>& linkWorld, double origin[3], double axis[3])
{
	if (j.parentLinkIdx < 0 || j.parentLinkIdx >= static_cast<int>(linkWorld.size()))
	{
		return false;
	}
	const double* W = linkWorld[static_cast<size_t>(j.parentLinkIdx)].data();
	const double ax = j.motion.axis[0];
	const double ay = j.motion.axis[1];
	const double az = j.motion.axis[2];
	const double ox = j.motion.originMm[0];
	const double oy = j.motion.originMm[1];
	const double oz = j.motion.originMm[2];
	origin[0] = W[0] * ox + W[4] * oy + W[8] * oz + W[12];
	origin[1] = W[1] * ox + W[5] * oy + W[9] * oz + W[13];
	origin[2] = W[2] * ox + W[6] * oy + W[10] * oz + W[14];
	axis[0] = W[0] * ax + W[4] * ay + W[8] * az;
	axis[1] = W[1] * ax + W[5] * ay + W[9] * az;
	axis[2] = W[2] * ax + W[6] * ay + W[10] * az;
	const double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
	if (n < 1e-12)
	{
		return false;
	}
	axis[0] /= n;
	axis[1] /= n;
	axis[2] /= n;
	return true;
}

bool closestPointsOnLines(const double o1[3], const double a1[3], const double o2[3], const double a2[3], double p1[3],
						  double p2[3])
{
	const double d0 = o1[0] - o2[0];
	const double d1 = o1[1] - o2[1];
	const double d2 = o1[2] - o2[2];
	const double a = a1[0] * a1[0] + a1[1] * a1[1] + a1[2] * a1[2];
	const double b = a1[0] * a2[0] + a1[1] * a2[1] + a1[2] * a2[2];
	const double c = a2[0] * a2[0] + a2[1] * a2[1] + a2[2] * a2[2];
	const double d = a1[0] * d0 + a1[1] * d1 + a1[2] * d2;
	const double e = a2[0] * d0 + a2[1] * d1 + a2[2] * d2;
	const double denom = a * c - b * b;
	double s = 0.0;
	double t = 0.0;
	if (std::abs(denom) < 1e-12)
	{
		s = 0.0;
		t = (c > 1e-12) ? (e / c) : 0.0;
	}
	else
	{
		s = (b * e - c * d) / denom;
		t = (a * e - b * d) / denom;
	}
	p1[0] = o1[0] + s * a1[0];
	p1[1] = o1[1] + s * a1[1];
	p1[2] = o1[2] + s * a1[2];
	p2[0] = o2[0] + t * a2[0];
	p2[1] = o2[1] + t * a2[1];
	p2[2] = o2[2] + t * a2[2];
	return true;
}

bool sphericalWristCenter(const kinematic_core::KinematicGraph& graph,
						  const std::vector<const kinematic_core::KinematicJoint*>& rev, double centerOut[3])
{
	if (rev.size() != 6)
	{
		return false;
	}
	double base[16];
	identity16(base);
	std::vector<double> q(6, 0.0);
	std::vector<std::array<double, 16>> linkWorld(graph.links.size());
	if (!kinematic_core::forwardKinematicsTree(graph, base, q.data(), q.size(),
											   reinterpret_cast<double(*)[16]>(linkWorld.data())))
	{
		return false;
	}
	double o[3][3];
	double a[3][3];
	for (int i = 0; i < 3; ++i)
	{
		if (!axisLineFromJoint(graph, *rev[static_cast<size_t>(3 + i)], linkWorld, o[i], a[i]))
		{
			return false;
		}
	}
	double p12a[3];
	double p12b[3];
	double p13a[3];
	double p13b[3];
	double p23a[3];
	double p23b[3];
	closestPointsOnLines(o[0], a[0], o[1], a[1], p12a, p12b);
	closestPointsOnLines(o[0], a[0], o[2], a[2], p13a, p13b);
	closestPointsOnLines(o[1], a[1], o[2], a[2], p23a, p23b);
	const auto dist = [](const double* u, const double* v)
	{
		const double dx = u[0] - v[0];
		const double dy = u[1] - v[1];
		const double dz = u[2] - v[2];
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	};
	if (dist(p12a, p12b) > kWristConcurMm || dist(p13a, p13b) > kWristConcurMm || dist(p23a, p23b) > kWristConcurMm)
	{
		return false;
	}
	centerOut[0] = (p12a[0] + p12b[0] + p13a[0] + p13b[0] + p23a[0] + p23b[0]) / 6.0;
	centerOut[1] = (p12a[1] + p12b[1] + p13a[1] + p13b[1] + p23a[1] + p23b[1]) / 6.0;
	centerOut[2] = (p12a[2] + p12b[2] + p13a[2] + p13b[2] + p23a[2] + p23b[2]) / 6.0;
	return true;
}

void normalizeQuat(osg::Quat& q)
{
	const double n2 = q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w();
	if (n2 <= 1e-24)
	{
		q.set(0.0, 0.0, 0.0, 1.0);
		return;
	}
	const double inv = 1.0 / std::sqrt(n2);
	q.set(q.x() * inv, q.y() * inv, q.z() * inv, q.w() * inv);
}

/// 相对姿态按 ZYZ 拆腕角（两支）
void zyzWristCandidates(const osg::Matrixd& R36, double outQ4[2], double outQ5[2], double outQ6[2])
{
	const double r02 = R36(0, 2);
	const double r12 = R36(1, 2);
	const double r22 = R36(2, 2);
	const double r20 = R36(2, 0);
	const double r21 = R36(2, 1);
	const double s5 = std::sqrt(r02 * r02 + r12 * r12);
	if (s5 > 1e-8)
	{
		outQ5[0] = std::atan2(s5, r22);
		outQ4[0] = std::atan2(r12, r02);
		outQ6[0] = std::atan2(r21, -r20);
		outQ5[1] = std::atan2(-s5, r22);
		outQ4[1] = std::atan2(-r12, -r02);
		outQ6[1] = std::atan2(-r21, r20);
	}
	else
	{
		outQ5[0] = (r22 >= 0.0) ? 0.0 : kPi;
		outQ4[0] = 0.0;
		outQ6[0] = std::atan2(-R36(0, 1), R36(0, 0));
		outQ5[1] = outQ5[0];
		outQ4[1] = outQ4[0];
		outQ6[1] = outQ6[0];
	}
}

bool measurePoseError(const QString& urdfPath, const QString& ikLink, const std::vector<double>& q,
					  const UrdfPoseIkTarget& target, double& posErr, double& rotErr)
{
	QVector<double> qQt;
	qQt.reserve(static_cast<int>(q.size()));
	for (double v : q)
	{
		qQt.push_back(v);
	}
	double pos[3] = {};
	double quat[4] = {};
	std::vector<double> J;
	if (!computeLinkPoseAndJacobianViaCore(urdfPath, qQt, ikLink, pos, quat, J, true, 1.0, nullptr))
	{
		return false;
	}
	const double dx = target.posMm[0] - pos[0];
	const double dy = target.posMm[1] - pos[1];
	const double dz = target.posMm[2] - pos[2];
	posErr = std::sqrt(dx * dx + dy * dy + dz * dz);
	rotErr = 0.0;
	if (target.hasOrientation)
	{
		osg::Quat cur(quat[0], quat[1], quat[2], quat[3]);
		osg::Quat des(target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]);
		normalizeQuat(cur);
		normalizeQuat(des);
		osg::Quat err = cur.inverse() * des;
		if (err.w() < 0.0)
		{
			err.set(-err.x(), -err.y(), -err.z(), -err.w());
		}
		const double vn = std::sqrt(err.x() * err.x() + err.y() * err.y() + err.z() * err.z());
		rotErr = (vn < 1e-12) ? 0.0 : 2.0 * std::atan2(vn, std::max(1e-12, err.w()));
	}
	return true;
}

IkConvergenceStatus classifyStatus(const UrdfIkSolverOptions& options, double posErr, double rotErr, bool useOri)
{
	const double posTol = options.positionToleranceMm > 0.0 ? options.positionToleranceMm : 0.5;
	const double rotTol =
		options.orientationToleranceRad > 0.0 ? options.orientationToleranceRad : 0.1 * kPi / 180.0;
	const double softRot =
		options.softOrientationToleranceRad > 0.0 ? options.softOrientationToleranceRad : 2.0 * kPi / 180.0;
	if (posErr <= posTol && (!useOri || rotErr <= rotTol))
	{
		return IkConvergenceStatus::HardConverged;
	}
	if (options.allowApproximateOrientation && posErr <= 1.0 && (!useOri || rotErr <= softRot))
	{
		return IkConvergenceStatus::SoftAccepted;
	}
	return IkConvergenceStatus::Failed;
}
} // namespace

bool SphericalWristAnalyticalIk::isEligible(const QString& urdfPath)
{
	kinematic_core::KinematicGraph graph;
	if (!buildUrdfKinematicGraph(urdfPath, graph, nullptr) || graph.dofCount() != 6)
	{
		return false;
	}
	std::vector<const kinematic_core::KinematicJoint*> rev;
	if (!collectRevoluteJoints(graph, rev) || rev.size() != 6)
	{
		return false;
	}
	double c[3] = {};
	return sphericalWristCenter(graph, rev, c);
}

std::vector<double> SphericalWristAnalyticalIk::solve(const QString& urdfPath, const QString& ikLink,
													  const UrdfPoseIkTarget& target, std::vector<double> seedJointRad,
													  const UrdfIkSolverOptions& options, std::string* failReason,
													  IkConvergenceStatus* status)
{
	if (status)
	{
		*status = IkConvergenceStatus::Failed;
	}
	if (!isEligible(urdfPath))
	{
		if (failReason)
		{
			*failReason = "not spherical-wrist 6R";
		}
		return {};
	}
	if (seedJointRad.size() != 6)
	{
		if (failReason)
		{
			*failReason = "analytical IK expects 6 DOF seed";
		}
		return {};
	}

	UrdfPoseIkTarget posOnly = target;
	posOnly.hasOrientation = false;
	UrdfIkSolverOptions posOpt = options;
	posOpt.maxIterations = std::max(options.maxIterations > 0 ? options.maxIterations : 80, 48);

	const auto seeds = expandPoseIkSeeds(seedJointRad);
	std::vector<std::vector<double>> accepted;
	std::string lastFail;

	for (const auto& seed : seeds)
	{
		std::string fr;
		std::vector<double> qArm =
			solveArmPoseViaKinematicCore(urdfPath, ikLink, posOnly, seed, posOpt, &fr);
		if (qArm.size() != 6)
		{
			if (!fr.empty())
			{
				lastFail = std::move(fr);
			}
			continue;
		}

		QVector<double> qZeroWrist;
		qZeroWrist.reserve(6);
		for (double v : qArm)
		{
			qZeroWrist.push_back(v);
		}
		qZeroWrist[3] = 0.0;
		qZeroWrist[4] = 0.0;
		qZeroWrist[5] = 0.0;
		double posArm[3] = {};
		double quatArm[4] = {};
		std::vector<double> Jtmp;
		if (!computeLinkPoseAndJacobianViaCore(urdfPath, qZeroWrist, ikLink, posArm, quatArm, Jtmp, true, 1.0, nullptr))
		{
			continue;
		}

		osg::Quat qArmOri(quatArm[0], quatArm[1], quatArm[2], quatArm[3]);
		normalizeQuat(qArmOri);
		osg::Matrixd R03;
		R03.setRotate(qArmOri);

		osg::Matrixd Rdes;
		if (target.hasOrientation)
		{
			osg::Quat qd(target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]);
			normalizeQuat(qd);
			Rdes.setRotate(qd);
		}
		else
		{
			Rdes = R03;
		}
		const osg::Matrixd R36 = osg::Matrixd::inverse(R03) * Rdes;

		double q4c[2];
		double q5c[2];
		double q6c[2];
		zyzWristCandidates(R36, q4c, q5c, q6c);
		for (int branch = 0; branch < 2; ++branch)
		{
			std::vector<double> q = qArm;
			q[3] = q4c[branch];
			q[4] = q5c[branch];
			q[5] = q6c[branch];
			double posErr = 0.0;
			double rotErr = 0.0;
			if (!measurePoseError(urdfPath, ikLink, q, target, posErr, rotErr))
			{
				continue;
			}
			const IkConvergenceStatus st = classifyStatus(options, posErr, rotErr, target.hasOrientation);
			if (st == IkConvergenceStatus::Failed)
			{
				continue;
			}
			accepted.push_back(std::move(q));
		}
	}

	if (accepted.empty())
	{
		if (failReason)
		{
			*failReason = lastFail.empty() ? "analytical spherical-wrist IK failed" : lastFail;
		}
		return {};
	}

	std::vector<double> best = selectNearestWrappedSolution(accepted, seedJointRad);
	double posErr = 0.0;
	double rotErr = 0.0;
	if (!measurePoseError(urdfPath, ikLink, best, target, posErr, rotErr))
	{
		if (failReason)
		{
			*failReason = "analytical FK check failed";
		}
		return {};
	}
	const IkConvergenceStatus st = classifyStatus(options, posErr, rotErr, target.hasOrientation);
	if (st == IkConvergenceStatus::Failed)
	{
		if (failReason)
		{
			*failReason = "analytical residual rejected";
		}
		return {};
	}
	if (status)
	{
		*status = st;
	}
	if (failReason)
	{
		failReason->clear();
	}
	return best;
}

} // namespace UrdfRobotLoader
