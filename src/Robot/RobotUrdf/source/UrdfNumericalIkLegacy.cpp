/// @file UrdfNumericalIkLegacy.cpp
/// @brief legacy BFS 雅可比 DLS，仅 SelfTest / 对照

#include "UrdfNumericalIkLegacy.h"

#include "UrdfKinematicsWorkspace.h"
#include "UrdfNumericalIk.h"
#include "UrdfRobotLoader.h"

#include <QVector>

#include <algorithm>
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
} // namespace

std::vector<double> solveArmPoseViaUrdfJacobianLegacy(const QString& urdfPath, const QString& ikLink,
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

	const int n = static_cast<int>(q.size());
	const bool useOrientation = target.hasOrientation;
	const int taskDim = useOrientation ? 6 : 3;
	const int iterLimit = options.maxIterations > 0 ? options.maxIterations : 180;
	const double lambda = options.lambda > 0.0 ? options.lambda : 1e-2;
	const double posTol = options.positionToleranceMm > 0.0 ? options.positionToleranceMm : 1e-2;
	const double rotTol =
		options.orientationToleranceRad > 0.0 ? options.orientationToleranceRad : 0.1 * 3.14159265358979323846 / 180.0;
	const double orientationWeight = useOrientation ? options.orientationWeight : 1.0;
	const double stepCap = options.maxJointStepRad > 0.0 ? options.maxJointStepRad : 0.2;

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

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		for (int j = 0; j < n; ++j)
		{
			qRad[j] = q[static_cast<size_t>(j)];
		}

		double pos[3] = {0.0, 0.0, 0.0};
		double quatXyzw[4] = {0.0, 0.0, 0.0, 1.0};
		if (!computeLinkPoseAndGeometricJacobian(urdfPath, qRad, ikLink, pos,
												 useOrientation ? quatXyzw : nullptr, ws.J, useOrientation,
												 orientationWeight, nullptr, &ws) ||
			static_cast<int>(ws.J.size()) < taskDim * n)
		{
			if (failReason)
			{
				*failReason = "URDF FK/Jacobian failed";
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
			osg::Quat curQuat(quatXyzw[0], quatXyzw[1], quatXyzw[2], quatXyzw[3]);
			normalizeQuatSafe(curQuat);
			double eRot[3] = {0.0, 0.0, 0.0};
			quatErrorAxisAngle(curQuat, targetQuat, eRot);
			ws.e[3] = eRot[0] * orientationWeight;
			ws.e[4] = eRot[1] * orientationWeight;
			ws.e[5] = eRot[2] * orientationWeight;
			rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		}

		if (posErr < bestPosErr)
		{
			bestPosErr = posErr;
			bestQ = q;
		}
		if (posErr < posTol && (!useOrientation || rotErr < rotTol))
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
	}

	if (bestPosErr < posTol * 10.0)
	{
		return bestQ;
	}
	if (failReason)
	{
		*failReason = "URDF DLS did not converge";
	}
	return {};
}

} // namespace UrdfRobotLoader
