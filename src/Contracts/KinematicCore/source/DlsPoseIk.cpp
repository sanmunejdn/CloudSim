#include "DlsPoseIk.h"

#include "GeometricJacobian.h"
#include "TreeForwardKinematics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace kinematic_core
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

void normalizeQuat(double q[4])
{
	const double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	if (n2 <= 1e-24)
	{
		q[0] = 0.0;
		q[1] = 0.0;
		q[2] = 0.0;
		q[3] = 1.0;
		return;
	}
	const double inv = 1.0 / std::sqrt(n2);
	q[0] *= inv;
	q[1] *= inv;
	q[2] *= inv;
	q[3] *= inv;
}

void quatFromRotationMatrix(const double m[16], double q[4])
{
	const double tr = m[0] + m[5] + m[10];
	if (tr > 0.0)
	{
		const double s = std::sqrt(tr + 1.0) * 2.0;
		q[3] = 0.25 * s;
		q[0] = (m[6] - m[9]) / s;
		q[1] = (m[8] - m[2]) / s;
		q[2] = (m[1] - m[4]) / s;
	}
	else if (m[0] > m[5] && m[0] > m[10])
	{
		const double s = std::sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0;
		q[3] = (m[6] - m[9]) / s;
		q[0] = 0.25 * s;
		q[1] = (m[1] + m[4]) / s;
		q[2] = (m[8] + m[2]) / s;
	}
	else if (m[5] > m[10])
	{
		const double s = std::sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0;
		q[3] = (m[8] - m[2]) / s;
		q[0] = (m[1] + m[4]) / s;
		q[1] = 0.25 * s;
		q[2] = (m[6] + m[9]) / s;
	}
	else
	{
		const double s = std::sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0;
		q[3] = (m[1] - m[4]) / s;
		q[0] = (m[8] + m[2]) / s;
		q[1] = (m[6] + m[9]) / s;
		q[2] = 0.25 * s;
	}
	normalizeQuat(q);
}

void quatConjugate(const double q[4], double out[4])
{
	out[0] = -q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = q[3];
}

void quatMultiply(const double a[4], const double b[4], double out[4])
{
	out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
	out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
	out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
	out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

bool quatErrorAxisAngle(const double from[4], const double to[4], double outErrRad[3])
{
	double qFrom[4] = {from[0], from[1], from[2], from[3]};
	double qTo[4] = {to[0], to[1], to[2], to[3]};
	normalizeQuat(qFrom);
	normalizeQuat(qTo);
	double qInv[4];
	quatConjugate(qFrom, qInv);
	double qErr[4];
	quatMultiply(qInv, qTo, qErr);
	if (qErr[3] < 0.0)
	{
		qErr[0] = -qErr[0];
		qErr[1] = -qErr[1];
		qErr[2] = -qErr[2];
		qErr[3] = -qErr[3];
	}
	const double vx = qErr[0];
	const double vy = qErr[1];
	const double vz = qErr[2];
	const double vNorm = std::sqrt(vx * vx + vy * vy + vz * vz);
	if (vNorm < 1e-12)
	{
		outErrRad[0] = 0.0;
		outErrRad[1] = 0.0;
		outErrRad[2] = 0.0;
		return true;
	}
	const double angle = 2.0 * std::atan2(vNorm, std::max(1e-12, qErr[3]));
	const double inv = 1.0 / vNorm;
	outErrRad[0] = vx * inv * angle;
	outErrRad[1] = vy * inv * angle;
	outErrRad[2] = vz * inv * angle;
	return true;
}

void clampJointAnglesToGraphLimits(const KinematicGraph& graph, std::vector<double>& q)
{
	for (const KinematicJoint& j : graph.joints)
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

bool linkPose(const KinematicGraph& graph, const double baseWorld[16], const std::vector<double>& q, int linkIdx,
			  double outPos[3], double outQuat[4])
{
	std::vector<std::array<double, 16>> linkWorld(graph.links.size());
	if (!forwardKinematicsTree(graph, baseWorld, q.data(), q.size(),
							   reinterpret_cast<double(*)[16]>(linkWorld.data())))
	{
		return false;
	}
	if (linkIdx < 0 || linkIdx >= static_cast<int>(linkWorld.size()))
	{
		return false;
	}
	const double* m = linkWorld[static_cast<size_t>(linkIdx)].data();
	outPos[0] = m[12];
	outPos[1] = m[13];
	outPos[2] = m[14];
	quatFromRotationMatrix(m, outQuat);
	return true;
}
} // namespace

DlsPoseIkResult solvePoseDampedLeastSquares(const KinematicGraph& graph, const double baseWorld[16],
											const int targetLinkIdx, const PoseIkTarget& target,
											std::vector<double>& qInOut, const DlsPoseIkOptions& opt)
{
	DlsPoseIkResult result;
	const int n = graph.dofCount();
	if (n <= 0 || static_cast<int>(qInOut.size()) < n)
	{
		return result;
	}

	const int taskDim = target.hasOrientation ? 6 : 3;
	double targetQuat[4] = {target.quatXyzw[0], target.quatXyzw[1], target.quatXyzw[2], target.quatXyzw[3]};
	if (target.hasOrientation)
	{
		normalizeQuat(targetQuat);
	}

	JacobianOptions jacOpt;
	jacOpt.orientationWeight = opt.orientationWeight;

	std::vector<double> bestQ = qInOut;
	double bestPosErr = 1e30;

	for (int iter = 0; iter < opt.maxIterations; ++iter)
	{
		double pos[3];
		double curQuat[4];
		if (!linkPose(graph, baseWorld, qInOut, targetLinkIdx, pos, curQuat))
		{
			break;
		}
		const double errX = target.positionMm[0] - pos[0];
		const double errY = target.positionMm[1] - pos[1];
		const double errZ = target.positionMm[2] - pos[2];
		result.positionErrorMm = std::sqrt(errX * errX + errY * errY + errZ * errZ);
		if (result.positionErrorMm < bestPosErr)
		{
			bestPosErr = result.positionErrorMm;
			bestQ = qInOut;
		}

		std::vector<double> errVec(static_cast<size_t>(taskDim), 0.0);
		errVec[0] = errX;
		errVec[1] = errY;
		errVec[2] = errZ;

		result.orientationErrorRad = 0.0;
		if (target.hasOrientation)
		{
			double eRot[3];
			quatErrorAxisAngle(curQuat, targetQuat, eRot);
			errVec[3] = eRot[0] * opt.orientationWeight;
			errVec[4] = eRot[1] * opt.orientationWeight;
			errVec[5] = eRot[2] * opt.orientationWeight;
			result.orientationErrorRad =
				std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		}

		if (result.positionErrorMm <= opt.positionToleranceMm &&
			(!target.hasOrientation || result.orientationErrorRad <= opt.orientationToleranceRad))
		{
			result.success = true;
			result.iterationsUsed = iter;
			return result;
		}

		std::vector<double> J;
		if (target.hasOrientation)
		{
			if (!computePoseJacobian(graph, baseWorld, qInOut.data(), qInOut.size(), targetLinkIdx, J, jacOpt))
			{
				break;
			}
		}
		else if (!computePositionJacobian(graph, baseWorld, qInOut.data(), qInOut.size(), targetLinkIdx, J, jacOpt))
		{
			break;
		}

		std::vector<double> JJt(static_cast<size_t>(taskDim * taskDim), 0.0);
		for (int r = 0; r < taskDim; ++r)
		{
			for (int c = 0; c < taskDim; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < n; ++k)
				{
					s += J[static_cast<size_t>(r * n + k)] * J[static_cast<size_t>(c * n + k)];
				}
				JJt[static_cast<size_t>(r * taskDim + c)] = s;
			}
		}
		for (int i = 0; i < taskDim; ++i)
		{
			JJt[static_cast<size_t>(i * taskDim + i)] += opt.lambdaDamping * opt.lambdaDamping;
		}
		if (!solveLinearSystem(JJt, errVec, taskDim))
		{
			break;
		}

		for (int j = 0; j < n; ++j)
		{
			double step = 0.0;
			for (int r = 0; r < taskDim; ++r)
			{
				step += J[static_cast<size_t>(r * n + j)] * errVec[static_cast<size_t>(r)];
			}
			if (step > opt.stepLimitRad)
			{
				step = opt.stepLimitRad;
			}
			if (step < -opt.stepLimitRad)
			{
				step = -opt.stepLimitRad;
			}
			qInOut[static_cast<size_t>(j)] += step;
		}
		clampJointAnglesToGraphLimits(graph, qInOut);
		result.iterationsUsed = iter + 1;
	}

	if (!result.success && bestPosErr <= opt.positionToleranceMm * 8.0)
	{
		qInOut = std::move(bestQ);
		result.positionErrorMm = bestPosErr;
		result.success = true;
		return result;
	}

	if (target.hasOrientation)
	{
		PoseIkTarget posOnly = target;
		posOnly.hasOrientation = false;
		DlsPoseIkOptions posOpt = opt;
		std::vector<double> qPos = qInOut;
		const DlsPoseIkResult posResult =
			solvePoseDampedLeastSquares(graph, baseWorld, targetLinkIdx, posOnly, qPos, posOpt);
		if (posResult.success)
		{
			qInOut = std::move(qPos);
			return posResult;
		}
	}
	return result;
}

} // namespace kinematic_core
