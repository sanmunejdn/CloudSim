/// @file UrdfNumericalIk.cpp
/// @brief URDF 臂位姿 DLS（复用 Workspace）

#include "UrdfNumericalIk.h"

#include "UrdfKinematicsWorkspace.h"
#include "UrdfRobotLoader.h"

#include <algorithm>
#include <cmath>
#include <osg/Quat>

namespace UrdfRobotLoader
{
namespace
{
bool solveLinearSystem(std::vector<double>& a, std::vector<double>& b, const int n)
{
	// 与历史 TeachIk Gauss-Jordan 一致，避免拖动短迭代下奇异/数值漂移
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
	// 须为 inv(from)*to；写成 to*inv(from) 会反号，拖动姿态 IK 发散
	if (!outErrRad)
	{
		return false;
	}
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

} // namespace

std::vector<double> solveArmPoseDampedLeastSquares(const QString& urdfPath, const QString& ikLink,
												   const UrdfPoseIkTarget& target, std::vector<double> q,
												   const UrdfIkSolverOptions& options, std::string* failReason)
{
	if (urdfPath.isEmpty() || ikLink.isEmpty() || q.empty())
	{
		if (failReason)
		{
			*failReason = "无URDF上下文";
		}
		return {};
	}
	const double targetPos[3] = {target.posMm[0], target.posMm[1], target.posMm[2]};
	const bool useOrientation = target.hasOrientation;
	osg::Quat targetQuat(target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]);
	normalizeQuatSafe(targetQuat);
	const double targetNormMm =
		std::sqrt(targetPos[0] * targetPos[0] + targetPos[1] * targetPos[1] + targetPos[2] * targetPos[2]);
	if (targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	UrdfKinematicsWorkspace& ws = threadLocalKinematicsWorkspace();
	const int n = static_cast<int>(q.size());
	const int taskDim = useOrientation ? 6 : 3;
	ws.ensureCapacity(n, 64, taskDim);

	double pos[3] = {0.0, 0.0, 0.0};
	osg::Quat curQuat;
	{
		ws.qRad.resize(n);
		for (int j = 0; j < n; ++j)
		{
			ws.qRad[j] = q[static_cast<size_t>(j)];
		}
		double quatXyZw[4] = {0.0, 0.0, 0.0, 1.0};
		if (!computeLinkPoseAndGeometricJacobian(urdfPath, ws.qRad, ikLink, pos, useOrientation ? quatXyZw : nullptr,
												 ws.J, useOrientation,
												 useOrientation ? options.orientationWeight : 1.0, nullptr, &ws))
		{
			if (failReason)
			{
				*failReason = "无URDF上下文";
			}
			return {};
		}
		if (useOrientation)
		{
			curQuat.set(quatXyZw[0], quatXyZw[1], quatXyZw[2], quatXyZw[3]);
			normalizeQuatSafe(curQuat);
		}
	}
	const double initialErr =
		std::sqrt((targetPos[0] - pos[0]) * (targetPos[0] - pos[0]) + (targetPos[1] - pos[1]) * (targetPos[1] - pos[1]) +
				  (targetPos[2] - pos[2]) * (targetPos[2] - pos[2]));
	if (initialErr > 10000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	const double lambda = options.lambda;
	const int iterLimit = options.maxIterations > 0 ? options.maxIterations : 180;
	const double orientationWeight = useOrientation ? options.orientationWeight : 1.0;
	const double stepCap = options.maxJointStepRad > 0.0 ? options.maxJointStepRad : 0.2;
	const double posTol = options.positionToleranceMm;
	const double rotTol = options.orientationToleranceRad;

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		ws.qRad.resize(n);
		for (int j = 0; j < n; ++j)
		{
			ws.qRad[j] = q[static_cast<size_t>(j)];
		}
		double quatXyZw[4] = {0.0, 0.0, 0.0, 1.0};
		if (!computeLinkPoseAndGeometricJacobian(urdfPath, ws.qRad, ikLink, pos, useOrientation ? quatXyZw : nullptr,
												 ws.J, useOrientation, orientationWeight, nullptr, &ws) ||
			static_cast<int>(ws.J.size()) < taskDim * n)
		{
			if (failReason)
			{
				*failReason = "无URDF上下文";
			}
			return {};
		}
		if (useOrientation)
		{
			curQuat.set(quatXyZw[0], quatXyZw[1], quatXyZw[2], quatXyZw[3]);
			normalizeQuatSafe(curQuat);
		}
		ws.e.assign(static_cast<size_t>(taskDim), 0.0);
		ws.e[0] = targetPos[0] - pos[0];
		ws.e[1] = targetPos[1] - pos[1];
		ws.e[2] = targetPos[2] - pos[2];
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
		if (posErr < posTol && (!useOrientation || rotErr < rotTol))
		{
			return q;
		}

		ws.jtj.assign(static_cast<size_t>(n * n), 0.0);
		ws.jte.assign(static_cast<size_t>(n), 0.0);
		for (int r = 0; r < taskDim; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				ws.jte[static_cast<size_t>(c)] += ws.J[static_cast<size_t>(r * n + c)] * ws.e[static_cast<size_t>(r)];
			}
		}
		for (int r = 0; r < n; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < taskDim; ++k)
				{
					s += ws.J[static_cast<size_t>(k * n + r)] * ws.J[static_cast<size_t>(k * n + c)];
				}
				ws.jtj[static_cast<size_t>(r * n + c)] = s;
			}
		}
		for (int i = 0; i < n; ++i)
		{
			ws.jtj[static_cast<size_t>(i * n + i)] += lambda * lambda;
		}
		if (!solveLinearSystem(ws.jtj, ws.jte, n))
		{
			if (failReason)
			{
				*failReason = "IK未收敛/超迭代";
			}
			return {};
		}
		for (int j = 0; j < n; ++j)
		{
			ws.jte[static_cast<size_t>(j)] =
				std::max(-stepCap, std::min(stepCap, ws.jte[static_cast<size_t>(j)]));
			q[static_cast<size_t>(j)] += ws.jte[static_cast<size_t>(j)];
		}
	}

	if (useOrientation)
	{
		UrdfPoseIkTarget posOnly = target;
		posOnly.hasOrientation = false;
		UrdfIkSolverOptions posOpt = options;
		posOpt.orientationWeight = 1.0;
		std::string posFail;
		std::vector<double> qPos =
			solveArmPoseDampedLeastSquares(urdfPath, ikLink, posOnly, q, posOpt, &posFail);
		if (!qPos.empty())
		{
			return qPos;
		}
	}

	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

} // namespace UrdfRobotLoader
