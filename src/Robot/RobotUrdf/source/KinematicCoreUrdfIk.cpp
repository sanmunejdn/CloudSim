#include "KinematicCoreUrdfIk.h"

#include "UrdfKinematicsWorkspace.h"
#include "UrdfRobotLoader.h"

#include "KinematicGraph.h"

#include <QVector>

#include <algorithm>
#include <array>
#include <cmath>
#include <osg/Quat>

namespace UrdfRobotLoader
{
namespace
{
bool solveLinearSystem(std::vector<double>& a, std::vector<double>& b, const int n)
{
	if (n <= 0 || static_cast<int>(a.size()) != n * n || static_cast<int>(b.size()) != n)
	{
		return false;
	}
	for (int col = 0; col < n; ++col)
	{
		int pivot = col;
		double maxAbs = std::abs(a[static_cast<size_t>(col * n + col)]);
		for (int r = col + 1; r < n; ++r)
		{
			const double v = std::abs(a[static_cast<size_t>(r * n + col)]);
			if (v > maxAbs)
			{
				maxAbs = v;
				pivot = r;
			}
		}
		if (maxAbs < 1e-12)
		{
			return false;
		}
		if (pivot != col)
		{
			for (int c = col; c < n; ++c)
			{
				std::swap(a[static_cast<size_t>(col * n + c)], a[static_cast<size_t>(pivot * n + c)]);
			}
			std::swap(b[static_cast<size_t>(col)], b[static_cast<size_t>(pivot)]);
		}
		const double diag = a[static_cast<size_t>(col * n + col)];
		for (int c = col; c < n; ++c)
		{
			a[static_cast<size_t>(col * n + c)] /= diag;
		}
		b[static_cast<size_t>(col)] /= diag;
		for (int r = 0; r < n; ++r)
		{
			if (r == col)
			{
				continue;
			}
			const double f = a[static_cast<size_t>(r * n + col)];
			if (std::abs(f) < 1e-15)
			{
				continue;
			}
			for (int c = col; c < n; ++c)
			{
				a[static_cast<size_t>(r * n + c)] -= f * a[static_cast<size_t>(col * n + c)];
			}
			b[static_cast<size_t>(r)] -= f * b[static_cast<size_t>(col)];
		}
	}
	return true;
}

void normalizeQuatSafe(osg::Quat& q)
{
	const double n2 = q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w();
	if (n2 <= 1e-24)
	{
		q.set(0.0, 0.0, 0.0, 1.0);
		return;
	}
	const double invN = 1.0 / std::sqrt(n2);
	q.set(q.x() * invN, q.y() * invN, q.z() * invN, q.w() * invN);
}

bool quatErrorAxisAngle(const osg::Quat& from, const osg::Quat& to, double outErrRad[3])
{
	osg::Quat qFrom = from;
	osg::Quat qTo = to;
	normalizeQuatSafe(qFrom);
	normalizeQuatSafe(qTo);
	osg::Quat qErr = qFrom.inverse() * qTo;
	if (qErr.w() < 0.0)
	{
		qErr.set(-qErr.x(), -qErr.y(), -qErr.z(), -qErr.w());
	}
	const double vx = qErr.x();
	const double vy = qErr.y();
	const double vz = qErr.z();
	const double vNorm = std::sqrt(vx * vx + vy * vy + vz * vz);
	if (vNorm < 1e-12)
	{
		outErrRad[0] = 0.0;
		outErrRad[1] = 0.0;
		outErrRad[2] = 0.0;
		return true;
	}
	const double angle = 2.0 * std::atan2(vNorm, std::max(1e-12, qErr.w()));
	const double inv = 1.0 / vNorm;
	outErrRad[0] = vx * inv * angle;
	outErrRad[1] = vy * inv * angle;
	outErrRad[2] = vz * inv * angle;
	return true;
}

void clampQToGraphLimits(const kinematic_core::KinematicGraph& graph, std::vector<double>& q)
{
	for (const kinematic_core::KinematicJoint& j : graph.joints)
	{
		if (j.qIndex < 0 || !j.motion.enabled || !j.motion.hasLimit ||
			static_cast<std::size_t>(j.qIndex) >= q.size())
		{
			continue;
		}
		q[static_cast<size_t>(j.qIndex)] =
			std::clamp(q[static_cast<size_t>(j.qIndex)], j.motion.lower, j.motion.upper);
	}
}

bool linkPoseFromGraph(const kinematic_core::KinematicGraph& graph, const int linkIdx, const QVector<double>& qRad,
					   double outPos[3], osg::Quat* outQuat)
{
	return computeLinkWorldPoseFromGraph(graph, linkIdx, qRad, outPos, outQuat, nullptr);
}

/// 一次 FK：tip（OSG）+ 几何雅可比
bool linkPoseAndJacobianFromGraph(const kinematic_core::KinematicGraph& graph, const int linkIdx,
								  const QVector<double>& qRad, double outPos[3], osg::Quat* outQuat,
								  std::vector<double>& outJ, const bool includeOrientation,
								  const double orientationWeight)
{
	return computeLinkWorldPoseAndJacobianFromGraph(graph, linkIdx, qRad, outPos, outQuat, outJ, includeOrientation,
													orientationWeight, nullptr);
}

/// 球腕：锁 J4–J6，仅用 J1–J3 收敛位置（TRAC-IK / KDL 球腕解耦思路）
std::vector<double> runArmOnlyPositionRefine(const QString& urdfPath, const QString& ikLink,
											 const kinematic_core::KinematicGraph& graph, const int linkIdx,
											 const UrdfPoseIkTarget& target, std::vector<double> q,
											 const UrdfIkSolverOptions& options)
{
	const int n = static_cast<int>(q.size());
	constexpr int kWristDof = 3;
	if (n <= kWristDof)
	{
		return {};
	}
	const int armEnd = n - kWristDof;
	const int iterLimit = options.maxIterations > 0 ? std::min(options.maxIterations, 80) : 80;
	const double lambda = options.lambda > 0.0 ? options.lambda : 1e-2;
	const double posTol = options.positionToleranceMm > 0.0 ? options.positionToleranceMm : 0.5;
	const double stepCap = options.maxJointStepRad > 0.0 ? options.maxJointStepRad : 0.25;
	constexpr double kSoftPosAcceptMm = 1.0;

	UrdfKinematicsWorkspace& ws = threadLocalKinematicsWorkspace();
	ws.ensureCapacity(n, 64, 3);

	QVector<double> qRad;
	qRad.resize(n);

	std::vector<double> bestQ = q;
	double bestPosErr = 1e30;

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		for (int j = 0; j < n; ++j)
		{
			qRad[j] = q[static_cast<size_t>(j)];
		}
		double pos[3] = {0.0, 0.0, 0.0};
		if (!linkPoseAndJacobianFromGraph(graph, linkIdx, qRad, pos, nullptr, ws.J, false, 1.0) ||
			static_cast<int>(ws.J.size()) < 3 * n)
		{
			break;
		}
		const double dx = target.posMm[0] - pos[0];
		const double dy = target.posMm[1] - pos[1];
		const double dz = target.posMm[2] - pos[2];
		const double posErr = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (posErr < bestPosErr)
		{
			bestPosErr = posErr;
			bestQ = q;
		}
		if (posErr <= posTol || posErr <= kSoftPosAcceptMm)
		{
			return q;
		}

		ws.e[0] = dx;
		ws.e[1] = dy;
		ws.e[2] = dz;

		std::vector<double> JJt(9, 0.0);
		std::vector<double> e3(3, 0.0);
		e3[0] = ws.e[0];
		e3[1] = ws.e[1];
		e3[2] = ws.e[2];
		for (int r = 0; r < 3; ++r)
		{
			for (int c = 0; c < 3; ++c)
			{
				double s = 0.0;
				for (int a = 0; a < armEnd; ++a)
				{
					s += ws.J[static_cast<size_t>(r * n + a)] * ws.J[static_cast<size_t>(c * n + a)];
				}
				JJt[static_cast<size_t>(r * 3 + c)] = s;
			}
			JJt[static_cast<size_t>(r * 3 + r)] += lambda * lambda;
		}
		if (!solveLinearSystem(JJt, e3, 3))
		{
			break;
		}

		for (int a = 0; a < armEnd; ++a)
		{
			double step = 0.0;
			for (int r = 0; r < 3; ++r)
			{
				step += ws.J[static_cast<size_t>(r * n + a)] * e3[static_cast<size_t>(r)];
			}
			if (step > stepCap)
			{
				step = stepCap;
			}
			if (step < -stepCap)
			{
				step = -stepCap;
			}
			q[static_cast<size_t>(a)] += step;
		}
		clampQToGraphLimits(graph, q);
	}

	if (bestPosErr <= kSoftPosAcceptMm)
	{
		return bestQ;
	}
	return {};
}

/// 球腕末三轴解耦：锁臂关节仅调姿态，避免 pos-then-ori 全臂 DLS 把位置拉偏
std::vector<double> runWristOnlyOrientationRefine(const QString& urdfPath, const QString& ikLink,
												  const kinematic_core::KinematicGraph& graph, const int linkIdx,
												  const UrdfPoseIkTarget& target, std::vector<double> q,
												  const UrdfIkSolverOptions& options)
{
	const int n = static_cast<int>(q.size());
	constexpr int kWristDof = 3;
	if (n < kWristDof || !target.hasOrientation)
	{
		return {};
	}
	const int wristStart = n - kWristDof;
	const int iterLimit = options.maxIterations > 0 ? std::min(options.maxIterations, 48) : 48;
	const double lambda = options.lambda > 0.0 ? options.lambda : 1e-2;
	const double rotTol =
		options.orientationToleranceRad > 0.0 ? options.orientationToleranceRad : 0.1 * 3.14159265358979323846 / 180.0;
	const double orientationWeight = options.orientationWeight > 0.0 ? options.orientationWeight : 300.0;
	const double stepCap = options.maxJointStepRad > 0.0 ? options.maxJointStepRad : 0.25;
	constexpr double kMaxPosDriftMm = 3.0;
	constexpr double kSoftRotAcceptRad = 2.0 * 3.14159265358979323846 / 180.0;

	osg::Quat targetQuat;
	targetQuat.set(target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]);
	normalizeQuatSafe(targetQuat);

	UrdfKinematicsWorkspace& ws = threadLocalKinematicsWorkspace();
	ws.ensureCapacity(n, 64, 6);

	QVector<double> qRad;
	qRad.resize(n);

	auto fkErrors = [&](const std::vector<double>& qIn, double& posErr, double& rotErr) -> bool {
		for (int j = 0; j < n; ++j)
		{
			qRad[j] = qIn[static_cast<size_t>(j)];
		}
		double pos[3] = {0.0, 0.0, 0.0};
		osg::Quat curQuat;
		if (!linkPoseFromGraph(graph, linkIdx, qRad, pos, &curQuat))
		{
			return false;
		}
		const double dx = target.posMm[0] - pos[0];
		const double dy = target.posMm[1] - pos[1];
		const double dz = target.posMm[2] - pos[2];
		posErr = std::sqrt(dx * dx + dy * dy + dz * dz);
		double eRot[3] = {0.0, 0.0, 0.0};
		quatErrorAxisAngle(curQuat, targetQuat, eRot);
		rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		return true;
	};

	double posErr0 = 0.0;
	double rotErr0 = 0.0;
	if (!fkErrors(q, posErr0, rotErr0))
	{
		return {};
	}
	if (posErr0 <= kMaxPosDriftMm && rotErr0 <= rotTol)
	{
		return q;
	}

	std::vector<double> bestQ = q;
	double bestRotErr = rotErr0;

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		for (int j = 0; j < n; ++j)
		{
			qRad[j] = q[static_cast<size_t>(j)];
		}
		double pos[3] = {0.0, 0.0, 0.0};
		osg::Quat curQuat;
		if (!linkPoseAndJacobianFromGraph(graph, linkIdx, qRad, pos, &curQuat, ws.J, true, orientationWeight) ||
			static_cast<int>(ws.J.size()) < 6 * n)
		{
			break;
		}

		const double dx = target.posMm[0] - pos[0];
		const double dy = target.posMm[1] - pos[1];
		const double dz = target.posMm[2] - pos[2];
		const double posErr = std::sqrt(dx * dx + dy * dy + dz * dz);
		double eRot[3] = {0.0, 0.0, 0.0};
		quatErrorAxisAngle(curQuat, targetQuat, eRot);
		const double rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		if (posErr > kMaxPosDriftMm)
		{
			break;
		}
		if (posErr <= kMaxPosDriftMm && rotErr <= rotTol)
		{
			return q;
		}
		if (rotErr < bestRotErr)
		{
			bestRotErr = rotErr;
			bestQ = q;
		}
		if (rotErr <= kSoftRotAcceptRad && posErr <= kMaxPosDriftMm)
		{
			return q;
		}

		std::vector<double> JJt(9, 0.0);
		std::vector<double> e3(3, 0.0);
		for (int r = 0; r < 3; ++r)
		{
			e3[static_cast<size_t>(r)] = eRot[r] * orientationWeight;
			for (int c = 0; c < 3; ++c)
			{
				double s = 0.0;
				for (int w = 0; w < kWristDof; ++w)
				{
					const int jCol = wristStart + w;
					s += ws.J[static_cast<size_t>((3 + r) * n + jCol)] *
						 ws.J[static_cast<size_t>((3 + c) * n + jCol)];
				}
				JJt[static_cast<size_t>(r * 3 + c)] = s;
			}
			JJt[static_cast<size_t>(r * 3 + r)] += lambda * lambda;
		}
		if (!solveLinearSystem(JJt, e3, 3))
		{
			break;
		}

		for (int w = 0; w < kWristDof; ++w)
		{
			const int jCol = wristStart + w;
			double step = 0.0;
			for (int r = 0; r < 3; ++r)
			{
				step += ws.J[static_cast<size_t>((3 + r) * n + jCol)] * e3[static_cast<size_t>(r)];
			}
			if (step > stepCap)
			{
				step = stepCap;
			}
			if (step < -stepCap)
			{
				step = -stepCap;
			}
			q[static_cast<size_t>(jCol)] += step;
		}
		clampQToGraphLimits(graph, q);
	}

	if (bestRotErr <= kSoftRotAcceptRad)
	{
		double posErr = 0.0;
		double rotErr = 0.0;
		if (fkErrors(bestQ, posErr, rotErr) && posErr <= kMaxPosDriftMm)
		{
			return bestQ;
		}
	}
	return {};
}

std::vector<double> runUrdfDlsLoop(const QString& urdfPath, const QString& ikLink,
								   const kinematic_core::KinematicGraph& graph, const int linkIdx,
								   const UrdfPoseIkTarget& target, std::vector<double> q,
								   const UrdfIkSolverOptions& options, std::string* failReason,
								   const bool allowPosThenOriRefine = true)
{
	const int n = static_cast<int>(q.size());
	const bool useOrientation = target.hasOrientation;
	const int taskDim = useOrientation ? 6 : 3;
	const int iterLimit = options.maxIterations > 0 ? options.maxIterations : 80;
	const double lambda = options.lambda > 0.0 ? options.lambda : 1e-2;
	const double posTol = options.positionToleranceMm > 0.0 ? options.positionToleranceMm : 0.5;
	const double rotTol =
		options.orientationToleranceRad > 0.0 ? options.orientationToleranceRad : 0.1 * 3.14159265358979323846 / 180.0;
	const double orientationWeight = useOrientation ? options.orientationWeight : 1.0;
	const double stepCap = options.maxJointStepRad > 0.0 ? options.maxJointStepRad : 0.25;

	QVector<double> qRad;
	qRad.reserve(n);
	for (double v : q)
	{
		qRad.push_back(v);
	}

	osg::Quat targetQuat;
	if (useOrientation)
	{
		targetQuat.set(target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]);
		normalizeQuatSafe(targetQuat);
	}

	UrdfKinematicsWorkspace& ws = threadLocalKinematicsWorkspace();
	ws.ensureCapacity(n, 64, taskDim);

	std::vector<double> bestQ = q;
	double bestPosErr = 1e30;
	double bestRotErr = 1e30;
	double bestCost = 1e30;
	// 循环内硬收敛仍用 options；软接受仅用于近收敛（禁止宽姿态兜底，否则与指令姿态脱节）
	constexpr double kSoftPosAcceptMm = 1.0;
	constexpr double kSoftRotAcceptRad = 2.0 * 3.14159265358979323846 / 180.0;

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		for (int j = 0; j < n; ++j)
		{
			qRad[j] = q[static_cast<size_t>(j)];
		}

		double pos[3] = {0.0, 0.0, 0.0};
		osg::Quat curQuat;
		if (!linkPoseAndJacobianFromGraph(graph, linkIdx, qRad, pos, useOrientation ? &curQuat : nullptr, ws.J,
										  useOrientation, orientationWeight) ||
			static_cast<int>(ws.J.size()) < taskDim * n)
		{
			if (failReason)
			{
				*failReason = useOrientation ? "URDF pose Jacobian failed" : "URDF position Jacobian failed";
			}
			break;
		}

		ws.e.assign(static_cast<size_t>(taskDim), 0.0);
		ws.e[0] = target.posMm[0] - pos[0];
		ws.e[1] = target.posMm[1] - pos[1];
		ws.e[2] = target.posMm[2] - pos[2];
		const double posErr = std::sqrt(ws.e[0] * ws.e[0] + ws.e[1] * ws.e[1] + ws.e[2] * ws.e[2]);

		double rotErr = 0.0;
		if (useOrientation)
		{
			double eRot[3] = {0.0, 0.0, 0.0};
			quatErrorAxisAngle(curQuat, targetQuat, eRot);
			ws.e[3] = eRot[0] * orientationWeight;
			ws.e[4] = eRot[1] * orientationWeight;
			ws.e[5] = eRot[2] * orientationWeight;
			rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		}

		const double cost = useOrientation ? (posErr + 200.0 * rotErr) : posErr;
		if (cost < bestCost)
		{
			bestCost = cost;
			bestPosErr = posErr;
			bestRotErr = rotErr;
			bestQ = q;
		}
		if (posErr <= posTol && (!useOrientation || rotErr <= rotTol))
		{
			return q;
		}

		std::vector<double> JJt(static_cast<size_t>(taskDim * taskDim), 0.0);
		for (int r = 0; r < taskDim; ++r)
		{
			for (int c = 0; c < taskDim; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < n; ++k)
				{
					s += ws.J[static_cast<size_t>(r * n + k)] * ws.J[static_cast<size_t>(c * n + k)];
				}
				JJt[static_cast<size_t>(r * taskDim + c)] = s;
			}
		}
		for (int i = 0; i < taskDim; ++i)
		{
			JJt[static_cast<size_t>(i * taskDim + i)] += lambda * lambda;
		}
		if (!solveLinearSystem(JJt, ws.e, taskDim))
		{
			break;
		}

		for (int j = 0; j < n; ++j)
		{
			double step = 0.0;
			for (int r = 0; r < taskDim; ++r)
			{
				step += ws.J[static_cast<size_t>(r * n + j)] * ws.e[static_cast<size_t>(r)];
			}
			if (step > stepCap)
			{
				step = stepCap;
			}
			if (step < -stepCap)
			{
				step = -stepCap;
			}
			q[static_cast<size_t>(j)] += step;
		}
		clampQToGraphLimits(graph, q);
	}

	// 仅位置目标：位置够近即可收
	if (!useOrientation)
	{
		if (bestPosErr <= posTol * 12.0)
		{
			return bestQ;
		}
		if (failReason)
		{
			*failReason = "KinematicCore DLS did not converge";
		}
		return {};
	}

	// 姿态已较好：软接受，避免 0.1° 硬阈把可用解打成失败
	if (bestPosErr <= kSoftPosAcceptMm && bestRotErr <= kSoftRotAcceptRad)
	{
		if (failReason)
		{
			failReason->clear();
		}
		return bestQ;
	}

	if (!allowPosThenOriRefine)
	{
		if (failReason)
		{
			*failReason = "KinematicCore DLS did not converge";
		}
		return {};
	}

	// 仅明显走错分支时跳过精修；86mm/11° 类近解仍需 pos-then-ori
	constexpr double kSkipRefinePosMm = 150.0;
	constexpr double kSkipRefineRotRad = 45.0 * 3.14159265358979323846 / 180.0;
	constexpr double kSkipRefinePosHardMm = 400.0;
	constexpr double kSkipRefineRotHardRad = 70.0 * 3.14159265358979323846 / 180.0;
	// 明显不可达或随机重启野种子：跳过昂贵 pos-then-ori（86mm 类近解仍保留）
	constexpr double kSkipRefinePosAloneMm = 150.0;
	const bool hopeless = bestPosErr > kSkipRefinePosHardMm || bestRotErr > kSkipRefineRotHardRad ||
						  (bestPosErr > kSkipRefinePosMm && bestRotErr > kSkipRefineRotRad) ||
						  bestPosErr > kSkipRefinePosAloneMm;
	if (hopeless)
	{
		if (failReason)
		{
			*failReason = "KinematicCore DLS did not converge";
		}
		return {};
	}

	// 姿态联立失败时：先到点再精修姿态。直接返回仅位置解会留下上百度姿态残差。
	int refineAttempts = 0;
	UrdfPoseIkTarget posOnly = target;
	posOnly.hasOrientation = false;
	UrdfIkSolverOptions posOpt = options;
	posOpt.maxIterations = std::max(iterLimit, 24);
	std::vector<double> qPos = runArmOnlyPositionRefine(urdfPath, ikLink, graph, linkIdx, posOnly, bestQ, posOpt);
	if (qPos.empty())
	{
		qPos = runUrdfDlsLoop(urdfPath, ikLink, graph, linkIdx, posOnly, bestQ, posOpt, failReason, false);
	}
	if (qPos.empty())
	{
		return {};
	}
	{
		QVector<double> qRad;
		qRad.reserve(n);
		for (double v : qPos)
		{
			qRad.push_back(v);
		}
		double pos[3] = {0.0, 0.0, 0.0};
		osg::Quat curQuat;
		osg::Quat targetQuatCheck;
		targetQuatCheck.set(target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]);
		normalizeQuatSafe(targetQuatCheck);
		if (linkPoseFromGraph(graph, linkIdx, qRad, pos, &curQuat))
		{
			const double dx = target.posMm[0] - pos[0];
			const double dy = target.posMm[1] - pos[1];
			const double dz = target.posMm[2] - pos[2];
			const double posErr = std::sqrt(dx * dx + dy * dy + dz * dz);
			double eRot[3] = {0.0, 0.0, 0.0};
			quatErrorAxisAngle(curQuat, targetQuatCheck, eRot);
			const double rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
			if (posErr <= kSoftPosAcceptMm && rotErr <= kSoftRotAcceptRad)
			{
				if (failReason)
				{
					failReason->clear();
				}
				return qPos;
			}
		}
	}
	UrdfIkSolverOptions oriOpt = options;
	oriOpt.maxIterations = std::max(iterLimit, 64);
	oriOpt.orientationToleranceRad = std::max(oriOpt.orientationToleranceRad, kSoftRotAcceptRad);
	oriOpt.positionToleranceMm = std::max(oriOpt.positionToleranceMm, 0.5);
	// 降权避免大姿态误差时旋转项淹没位置项导致发散
	if (oriOpt.orientationWeight > 50.0)
	{
		oriOpt.orientationWeight = 50.0;
	}
	// 腕部多解：扫末轴 + 翻转 J5
	constexpr double kPi = 3.14159265358979323846;
	const int jTail = n - 1;
	const int jWrist = n >= 2 ? n - 2 : -1;
	const double qTail0 = (jTail >= 0) ? qPos[static_cast<size_t>(jTail)] : 0.0;
	const double qWrist0 = (jWrist >= 0) ? qPos[static_cast<size_t>(jWrist)] : 0.0;
	const double kTailOffsets[] = {0.0,	 0.25 * kPi, -0.25 * kPi, 0.5 * kPi, -0.5 * kPi,
								   0.75 * kPi, -0.75 * kPi, kPi,	   -kPi};
	const int maxRefineAttempts =
		options.maxPosThenOriAttempts > 0 ? options.maxPosThenOriAttempts : 18;
	std::string refineFail;
	for (int wristFlip = 0; wristFlip < 2; ++wristFlip)
	{
		for (double off : kTailOffsets)
		{
			if (refineAttempts >= maxRefineAttempts)
			{
				break;
			}
			std::vector<double> qSeed = qPos;
			if (jWrist >= 0 && wristFlip != 0)
			{
				qSeed[static_cast<size_t>(jWrist)] = -qWrist0;
			}
			if (jTail >= 0)
			{
				qSeed[static_cast<size_t>(jTail)] = qTail0 + off;
			}
			clampQToGraphLimits(graph, qSeed);
			++refineAttempts;
			std::vector<double> qOri = runWristOnlyOrientationRefine(urdfPath, ikLink, graph, linkIdx, target, qSeed,
																	 oriOpt);
			if (qOri.empty())
			{
				qOri = runUrdfDlsLoop(urdfPath, ikLink, graph, linkIdx, target, std::move(qSeed), oriOpt, &refineFail,
									  false);
			}
			if (!qOri.empty())
			{
				if (failReason)
				{
					failReason->clear();
				}
				return qOri;
			}
		}
		if (refineAttempts >= maxRefineAttempts)
		{
			break;
		}
	}
	if (failReason)
	{
		*failReason = refineFail.empty() ? "KinematicCore DLS did not converge" : refineFail;
	}
	return {};
}
} // namespace

bool computeLinkPoseAndJacobianViaCore(const QString& urdfPath, const QVector<double>& jointAnglesRad,
									   const QString& linkName, double outPosMm[3], double* outQuatXyzw,
									   std::vector<double>& outJ_rowMajor, const bool includeOrientation,
									   const double orientationWeight, QString* errorMessage)
{
	kinematic_core::KinematicGraph graph;
	if (!buildUrdfKinematicGraph(urdfPath, graph, errorMessage))
	{
		return false;
	}
	const int linkIdx = graph.linkIndexById(linkName.toStdString());
	if (linkIdx < 0)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Link '%1' not in graph.").arg(linkName);
		}
		return false;
	}
	const int n = graph.dofCount();
	if (jointAnglesRad.size() < n)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("joint angle count mismatch.");
		}
		return false;
	}

	osg::Quat quat;
	if (!linkPoseAndJacobianFromGraph(graph, linkIdx, jointAnglesRad, outPosMm, outQuatXyzw ? &quat : nullptr,
									  outJ_rowMajor, includeOrientation, orientationWeight))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("link pose + Jacobian failed.");
		}
		return false;
	}
	if (outQuatXyzw)
	{
		outQuatXyzw[0] = quat.x();
		outQuatXyzw[1] = quat.y();
		outQuatXyzw[2] = quat.z();
		outQuatXyzw[3] = quat.w();
	}
	const int need = includeOrientation ? 6 * n : 3 * n;
	return static_cast<int>(outJ_rowMajor.size()) >= need;
}

std::vector<double> solveArmPoseViaKinematicCore(const QString& urdfPath, const QString& ikLink,
												 const UrdfPoseIkTarget& target, std::vector<double> q,
												 const UrdfIkSolverOptions& options, std::string* failReason)
{
	kinematic_core::KinematicGraph graph;
	if (!buildUrdfKinematicGraph(urdfPath, graph, nullptr))
	{
		if (failReason)
		{
			*failReason = "buildUrdfKinematicGraph failed";
		}
		return {};
	}
	const int linkIdx = graph.linkIndexById(ikLink.toStdString());
	if (linkIdx < 0)
	{
		if (failReason)
		{
			*failReason = "ik link not in graph";
		}
		return {};
	}
	clampQToGraphLimits(graph, q);
	return runUrdfDlsLoop(urdfPath, ikLink, graph, linkIdx, target, std::move(q), options, failReason);
}

} // namespace UrdfRobotLoader
