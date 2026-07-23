/// @file RobotTeachIk.cpp
/// @brief RobotTeachIk 实现

#include "RobotTeachIk.h"

#include "RobotCoordinateFrames.h"
#include "RobotMatrixOsgBridge.h"

#include <QHash>
#include <QString>
#include <algorithm>
#include <cmath>
#include <utility>

#include <Adapters.h>
#include <ToolKinematics.h>
#include <UrdfRobotLoader.h>
#include <osg/Quat>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

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

bool solveLinearSystem(std::vector<double>& a, std::vector<double>& b, int n)
{
	if (n <= 0 || static_cast<int>(a.size()) != n * n || static_cast<int>(b.size()) != n)
	{
		return false;
	}
	for (int col = 0; col < n; ++col)
	{
		int pivot = col;
		double maxAbs = std::abs(a[col * n + col]);
		for (int r = col + 1; r < n; ++r)
		{
			const double v = std::abs(a[r * n + col]);
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
				std::swap(a[col * n + c], a[pivot * n + c]);
			}
			std::swap(b[col], b[pivot]);
		}
		const double diag = a[col * n + col];
		for (int c = col; c < n; ++c)
		{
			a[col * n + c] /= diag;
		}
		b[col] /= diag;
		for (int r = 0; r < n; ++r)
		{
			if (r == col)
			{
				continue;
			}
			const double f = a[r * n + col];
			if (std::abs(f) < 1e-15)
			{
				continue;
			}
			for (int c = col; c < n; ++c)
			{
				a[r * n + c] -= f * a[col * n + c];
			}
			b[r] -= f * b[col];
		}
	}
	return true;
}

bool quatErrorAxisAngle(const osg::Quat& from, const osg::Quat& to, double outErrRad[3])
{
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

struct IkLinkTarget
{
	double pos[3] = {0.0, 0.0, 0.0};
	osg::Quat quat;
	bool hasOrientation = false;
};

bool ikLinkTargetFromTeachContext(const RobotTeachIk::TeachIkContext& ctx, IkLinkTarget& out)
{
	const engine::RigidTransform T_flange_tool = RobotCoordinate::rigidTransformFromBackendMat4(ctx.T_flange_tool);
	const engine::RigidTransform T_base_link = engine::flangeFromToolOrigin(ctx.T_base_target, T_flange_tool);
	double tx = 0.0;
	double ty = 0.0;
	double tz = 0.0;
	T_base_link.translationMm(tx, ty, tz);
	out.pos[0] = tx;
	out.pos[1] = ty;
	out.pos[2] = tz;
	if (ctx.useOrientation)
	{
		const Eigen::Quaterniond q = T_base_link.rotation().normalized();
		out.quat.set(q.x(), q.y(), q.z(), q.w());
		normalizeQuatSafe(out.quat);
		out.hasOrientation = true;
	}
	else
	{
		out.hasOrientation = false;
	}
	return true;
}

bool linkPoseFromUrdf(const QString& urdfPath, const QString& linkName, const std::vector<double>& q, double outPos[3],
					  osg::Quat* outRot = nullptr)
{
	QVector<double> qQt;
	qQt.reserve(static_cast<int>(q.size()));
	for (double v : q)
	{
		qQt.push_back(v);
	}
	QHash<QString, osg::Matrixd> linkWorldByName;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, qQt, linkWorldByName, nullptr))
	{
		return false;
	}
	if (!linkWorldByName.contains(linkName))
	{
		return false;
	}
	const osg::Vec3d t = linkWorldByName.value(linkName).getTrans();
	outPos[0] = t.x();
	outPos[1] = t.y();
	outPos[2] = t.z();
	if (outRot)
	{
		*outRot = linkWorldByName.value(linkName).getRotate();
		normalizeQuatSafe(*outRot);
	}
	return true;
}

std::vector<double> solveUrdfNumericalIk(const QString& urdfPath, const QString& ikLink, const IkLinkTarget& linkTarget,
										 std::vector<double> q, const int maxIters, std::string* failReason)
{
	if (urdfPath.isEmpty() || ikLink.isEmpty() || q.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double target[3] = {linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2]};
	const bool useOrientation = linkTarget.hasOrientation;
	const osg::Quat targetQuat = useOrientation ? linkTarget.quat : osg::Quat();
	const double targetNormMm = std::sqrt(target[0] * target[0] + target[1] * target[1] + target[2] * target[2]);
	if (targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	double pos[3] = {0.0, 0.0, 0.0};
	osg::Quat curQuat;
	if (!linkPoseFromUrdf(urdfPath, ikLink, q, pos, useOrientation ? &curQuat : nullptr))
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double initialErr =
		std::sqrt((target[0] - pos[0]) * (target[0] - pos[0]) + (target[1] - pos[1]) * (target[1] - pos[1]) +
				  (target[2] - pos[2]) * (target[2] - pos[2]));
	if (initialErr > 10000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	const double eps = 1e-4;
	const double lambda = 1e-2;
	const int taskDim = useOrientation ? 6 : 3;
	const int iterLimit = maxIters > 0 ? maxIters : 180;
	const double orientationWeight = useOrientation ? 300.0 : 1.0;
	for (int iter = 0; iter < iterLimit; ++iter)
	{
		if (!linkPoseFromUrdf(urdfPath, ikLink, q, pos, useOrientation ? &curQuat : nullptr))
		{
			if (failReason)
			{
				*failReason = "无DH上下文";
			}
			return {};
		}
		std::vector<double> e(static_cast<size_t>(taskDim), 0.0);
		e[0] = target[0] - pos[0];
		e[1] = target[1] - pos[1];
		e[2] = target[2] - pos[2];
		const double posErr = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
		double rotErr = 0.0;
		if (useOrientation)
		{
			double eRot[3] = {0.0, 0.0, 0.0};
			quatErrorAxisAngle(curQuat, targetQuat, eRot);
			e[3] = eRot[0] * orientationWeight;
			e[4] = eRot[1] * orientationWeight;
			e[5] = eRot[2] * orientationWeight;
			rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		}
		if (posErr < 1e-2 && (!useOrientation || rotErr < 0.1 * kDegToRad))
		{
			return q;
		}

		const int n = static_cast<int>(q.size());
		std::vector<double> J(static_cast<size_t>(taskDim * n), 0.0);
		for (int j = 0; j < n; ++j)
		{
			std::vector<double> qPert = q;
			qPert[static_cast<size_t>(j)] += eps;
			double p2[3] = {0.0, 0.0, 0.0};
			osg::Quat q2;
			if (!linkPoseFromUrdf(urdfPath, ikLink, qPert, p2, useOrientation ? &q2 : nullptr))
			{
				if (failReason)
				{
					*failReason = "无DH上下文";
				}
				return {};
			}
			J[0 * n + j] = (p2[0] - pos[0]) / eps;
			J[1 * n + j] = (p2[1] - pos[1]) / eps;
			J[2 * n + j] = (p2[2] - pos[2]) / eps;
			if (useOrientation)
			{
				double dRot[3] = {0.0, 0.0, 0.0};
				quatErrorAxisAngle(curQuat, q2, dRot);
				J[3 * n + j] = (dRot[0] / eps) * orientationWeight;
				J[4 * n + j] = (dRot[1] / eps) * orientationWeight;
				J[5 * n + j] = (dRot[2] / eps) * orientationWeight;
			}
		}

		std::vector<double> jtj(static_cast<size_t>(n * n), 0.0);
		std::vector<double> jte(static_cast<size_t>(n), 0.0);
		for (int r = 0; r < taskDim; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				jte[static_cast<size_t>(c)] += J[static_cast<size_t>(r * n + c)] * e[static_cast<size_t>(r)];
			}
		}
		for (int r = 0; r < n; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < taskDim; ++k)
				{
					s += J[static_cast<size_t>(k * n + r)] * J[static_cast<size_t>(k * n + c)];
				}
				jtj[static_cast<size_t>(r * n + c)] = s;
			}
		}
		for (int i = 0; i < n; ++i)
		{
			jtj[static_cast<size_t>(i * n + i)] += lambda * lambda;
		}
		if (!solveLinearSystem(jtj, jte, n))
		{
			if (failReason)
			{
				*failReason = "IK未收敛/超迭代";
			}
			return {};
		}
		for (int j = 0; j < n; ++j)
		{
			jte[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jte[static_cast<size_t>(j)]));
			q[static_cast<size_t>(j)] += jte[static_cast<size_t>(j)];
		}
	}

	if (useOrientation)
	{
		const double posTarget[3] = {linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2]};
		std::vector<double> qPos = q;
		for (int iter = 0; iter < iterLimit; ++iter)
		{
			if (!linkPoseFromUrdf(urdfPath, ikLink, qPos, pos, nullptr))
			{
				break;
			}
			const double e0 = posTarget[0] - pos[0];
			const double e1 = posTarget[1] - pos[1];
			const double e2 = posTarget[2] - pos[2];
			const double posErr = std::sqrt(e0 * e0 + e1 * e1 + e2 * e2);
			if (posErr < 1e-2)
			{
				return qPos;
			}
			const int n = static_cast<int>(qPos.size());
			std::vector<double> J(static_cast<size_t>(3 * n), 0.0);
			for (int j = 0; j < n; ++j)
			{
				std::vector<double> qPert = qPos;
				qPert[static_cast<size_t>(j)] += eps;
				double p2[3] = {0.0, 0.0, 0.0};
				if (!linkPoseFromUrdf(urdfPath, ikLink, qPert, p2, nullptr))
				{
					break;
				}
				J[0 * n + j] = (p2[0] - pos[0]) / eps;
				J[1 * n + j] = (p2[1] - pos[1]) / eps;
				J[2 * n + j] = (p2[2] - pos[2]) / eps;
			}
			std::vector<double> jtj(static_cast<size_t>(n * n), 0.0);
			std::vector<double> jte(static_cast<size_t>(n), 0.0);
			const double e[3] = {e0, e1, e2};
			for (int r = 0; r < 3; ++r)
			{
				for (int c = 0; c < n; ++c)
				{
					jte[static_cast<size_t>(c)] += J[static_cast<size_t>(r * n + c)] * e[r];
				}
			}
			for (int r = 0; r < n; ++r)
			{
				for (int c = 0; c < n; ++c)
				{
					double s = 0.0;
					for (int k = 0; k < 3; ++k)
					{
						s += J[static_cast<size_t>(k * n + r)] * J[static_cast<size_t>(k * n + c)];
					}
					jtj[static_cast<size_t>(r * n + c)] = s;
				}
			}
			for (int i = 0; i < n; ++i)
			{
				jtj[static_cast<size_t>(i * n + i)] += lambda * lambda;
			}
			if (!solveLinearSystem(jtj, jte, n))
			{
				break;
			}
			for (int j = 0; j < n; ++j)
			{
				jte[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jte[static_cast<size_t>(j)]));
				qPos[static_cast<size_t>(j)] += jte[static_cast<size_t>(j)];
			}
		}
	}

	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

std::vector<double> solveUrdfNumericalIkCoupledExternal(const QString& urdfPath, const QString& ikLink,
														const IkLinkTarget& linkTarget, std::vector<double> q,
														double& qExtInOut, const double axis[3], const double lower,
														const double upper, const int maxIters, std::string* failReason,
														const double externalDeltaPriorWeight = 0.0,
														const bool adaptiveExternalDamping = true,
														const double qExtSeed = 0.0)
{
	if (urdfPath.isEmpty() || ikLink.isEmpty() || q.empty() || !axis)
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double target[3] = {linkTarget.pos[0], linkTarget.pos[1], linkTarget.pos[2]};
	const bool useOrientation = linkTarget.hasOrientation;
	const osg::Quat targetQuat = useOrientation ? linkTarget.quat : osg::Quat();
	const double eps = 1e-4;
	const double lambda = 1e-2;
	const int taskDim = useOrientation ? 6 : 3;
	const int iterLimit = maxIters > 0 ? maxIters : 180;
	const double orientationWeight = useOrientation ? 300.0 : 1.0;
	const int nArm = static_cast<int>(q.size());
	const int n = nArm + 1;
	qExtInOut = std::clamp(qExtInOut, lower, upper);
	const double qeSeedClamped = std::clamp(qExtSeed, lower, upper);

	std::vector<double> bestQ = q;
	double bestQe = qExtInOut;
	double bestPosErr = 1e30;

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		double pos[3] = {0.0, 0.0, 0.0};
		osg::Quat curQuat;
		if (!linkPoseFromUrdf(urdfPath, ikLink, q, pos, useOrientation ? &curQuat : nullptr))
		{
			if (failReason)
			{
				*failReason = "无DH上下文";
			}
			return {};
		}
		// p_eff = FK + q_e * axis
		const double pEff[3] = {pos[0] + qExtInOut * axis[0], pos[1] + qExtInOut * axis[1],
								pos[2] + qExtInOut * axis[2]};
		std::vector<double> e(static_cast<size_t>(taskDim), 0.0);
		e[0] = target[0] - pEff[0];
		e[1] = target[1] - pEff[1];
		e[2] = target[2] - pEff[2];
		const double posErr = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
		double rotErr = 0.0;
		if (useOrientation)
		{
			double eRot[3] = {0.0, 0.0, 0.0};
			quatErrorAxisAngle(curQuat, targetQuat, eRot);
			e[3] = eRot[0] * orientationWeight;
			e[4] = eRot[1] * orientationWeight;
			e[5] = eRot[2] * orientationWeight;
			rotErr = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		}
		if (posErr < bestPosErr)
		{
			bestPosErr = posErr;
			bestQ = q;
			bestQe = qExtInOut;
		}
		if (posErr < 1e-2 && (!useOrientation || rotErr < 0.1 * kDegToRad))
		{
			qExtInOut = bestQe;
			return bestQ;
		}

		std::vector<double> J(static_cast<size_t>(taskDim * n), 0.0);
		for (int j = 0; j < nArm; ++j)
		{
			std::vector<double> qPert = q;
			qPert[static_cast<size_t>(j)] += eps;
			double p2[3] = {0.0, 0.0, 0.0};
			osg::Quat q2;
			if (!linkPoseFromUrdf(urdfPath, ikLink, qPert, p2, useOrientation ? &q2 : nullptr))
			{
				if (failReason)
				{
					*failReason = "无DH上下文";
				}
				return {};
			}
			J[0 * n + j] = (p2[0] - pos[0]) / eps;
			J[1 * n + j] = (p2[1] - pos[1]) / eps;
			J[2 * n + j] = (p2[2] - pos[2]) / eps;
			if (useOrientation)
			{
				double dRot[3] = {0.0, 0.0, 0.0};
				quatErrorAxisAngle(curQuat, q2, dRot);
				J[3 * n + j] = (dRot[0] / eps) * orientationWeight;
				J[4 * n + j] = (dRot[1] / eps) * orientationWeight;
				J[5 * n + j] = (dRot[2] / eps) * orientationWeight;
			}
		}
		// 外轴列：∂p_eff/∂q_e = axis
		J[0 * n + nArm] = axis[0];
		J[1 * n + nArm] = axis[1];
		J[2 * n + nArm] = axis[2];

		// 误差沿轨占比 + 臂沿轨能力 → 外轴阻尼：正交误差偏臂，沿轨且臂弱则放宽外轴
		double lambdaExtScale = 1.0;
		if (adaptiveExternalDamping && posErr > 1e-6)
		{
			const double ePar = e[0] * axis[0] + e[1] * axis[1] + e[2] * axis[2];
			const double railShare = std::min(1.0, std::abs(ePar) / posErr);
			double armAlong = 0.0;
			for (int j = 0; j < nArm; ++j)
			{
				const double jPar = J[0 * n + j] * axis[0] + J[1 * n + j] * axis[1] + J[2 * n + j] * axis[2];
				armAlong += jPar * jPar;
			}
			armAlong = std::sqrt(armAlong);
			const double armWeak = 1.0 / (1.0 + armAlong); // 臂沿轨雅可比弱 → 接近 1
			lambdaExtScale = 0.25 + 2.5 * (1.0 - railShare) + 1.5 * (1.0 - armWeak) * (1.0 - railShare);
		}

		std::vector<double> jtj(static_cast<size_t>(n * n), 0.0);
		std::vector<double> jte(static_cast<size_t>(n), 0.0);
		for (int r = 0; r < taskDim; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				jte[static_cast<size_t>(c)] += J[static_cast<size_t>(r * n + c)] * e[static_cast<size_t>(r)];
			}
		}
		for (int r = 0; r < n; ++r)
		{
			for (int c = 0; c < n; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < taskDim; ++k)
				{
					s += J[static_cast<size_t>(k * n + r)] * J[static_cast<size_t>(k * n + c)];
				}
				jtj[static_cast<size_t>(r * n + c)] = s;
			}
		}
		for (int i = 0; i < nArm; ++i)
		{
			jtj[static_cast<size_t>(i * n + i)] += lambda * lambda;
		}
		jtj[static_cast<size_t>(nArm * n + nArm)] += (lambda * lambdaExtScale) * (lambda * lambdaExtScale);
		if (externalDeltaPriorWeight > 0.0)
		{
			jtj[static_cast<size_t>(nArm * n + nArm)] += externalDeltaPriorWeight;
			jte[static_cast<size_t>(nArm)] -= externalDeltaPriorWeight * (qExtInOut - qeSeedClamped);
		}
		if (!solveLinearSystem(jtj, jte, n))
		{
			if (failReason)
			{
				*failReason = "IK未收敛/超迭代";
			}
			if (bestPosErr < 1e29)
			{
				qExtInOut = bestQe;
				return bestQ;
			}
			return {};
		}
		for (int j = 0; j < nArm; ++j)
		{
			jte[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jte[static_cast<size_t>(j)]));
			q[static_cast<size_t>(j)] += jte[static_cast<size_t>(j)];
		}
		const double dqExt = std::max(-50.0, std::min(50.0, jte[static_cast<size_t>(nArm)]));
		qExtInOut = std::clamp(qExtInOut + dqExt, lower, upper);
	}

	if (bestPosErr < 1e29)
	{
		qExtInOut = bestQe;
		return bestQ;
	}
	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

double residualToolOriginMm(const RobotTeachIk::TeachIkContext& ctx, const std::vector<double>& q)
{
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromTeachContext(ctx, linkTarget))
	{
		return 1e30;
	}
	double toolOriginPos[3]{};
	ctx.T_base_target.translationMm(toolOriginPos[0], toolOriginPos[1], toolOriginPos[2]);
	if (ctx.externalAxis.enabled)
	{
		toolOriginPos[0] -= ctx.externalAxis.qExternal * ctx.externalAxis.axis[0];
		toolOriginPos[1] -= ctx.externalAxis.qExternal * ctx.externalAxis.axis[1];
		toolOriginPos[2] -= ctx.externalAxis.qExternal * ctx.externalAxis.axis[2];
	}

	const engine::RigidTransform T_flange_tool = RobotCoordinate::rigidTransformFromBackendMat4(ctx.T_flange_tool);
	QVector<double> qQt;
	qQt.reserve(static_cast<int>(q.size()));
	for (double v : q)
	{
		qQt.push_back(v);
	}
	QHash<QString, osg::Matrixd> linkWorld;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(ctx.urdfPath, qQt, linkWorld, nullptr) ||
		!linkWorld.contains(ctx.ikLinkName))
	{
		return 1e30;
	}
	const BackendMat4 fkTcpMat =
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(ctx.ikLinkName), ctx.T_flange_tool);
	const RobotCoordinate::RobotRigidFrame fkTcpFrame = RobotCoordinate::mat4ToFrame(fkTcpMat);
	const double fkPos[3] = {fkTcpFrame.positionMm[0], fkTcpFrame.positionMm[1], fkTcpFrame.positionMm[2]};
	const double dx = fkPos[0] - toolOriginPos[0];
	const double dy = fkPos[1] - toolOriginPos[1];
	const double dz = fkPos[2] - toolOriginPos[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}
} // namespace

namespace RobotTeachIk
{
void applyExternalAxisToTargetPos(const double axis[3], const double qExt, double posInOut[3])
{
	if (!axis || !posInOut)
	{
		return;
	}
	posInOut[0] -= qExt * axis[0];
	posInOut[1] -= qExt * axis[1];
	posInOut[2] -= qExt * axis[2];
}

TeachIkResult solveTeachIk(const TeachIkContext& ctx)
{
	TeachIkResult out;
	if (ctx.urdfPath.isEmpty() || ctx.ikLinkName.isEmpty() || ctx.seedJointRad.empty())
	{
		out.error = "无DH上下文";
		return out;
	}
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromTeachContext(ctx, linkTarget))
	{
		out.error = "无效目标位姿";
		return out;
	}
	std::string failReason;
	const int maxIters = ctx.maxIkIterations > 0 ? ctx.maxIkIterations : 180;
	std::vector<double> q;
	double qExt = ctx.externalAxis.qExternal;
	if (ctx.externalAxis.enabled && ctx.externalAxis.optimizeExternal)
	{
		qExt = std::clamp(qExt, ctx.externalAxis.lower, ctx.externalAxis.upper);
		q = solveUrdfNumericalIkCoupledExternal(ctx.urdfPath, ctx.ikLinkName, linkTarget, ctx.seedJointRad, qExt,
												ctx.externalAxis.axis, ctx.externalAxis.lower, ctx.externalAxis.upper,
												maxIters, &failReason, ctx.externalAxis.externalDeltaPriorWeight,
												ctx.externalAxis.adaptiveExternalDamping, qExt);
	}
	else
	{
		if (ctx.externalAxis.enabled)
		{
			qExt = std::clamp(qExt, ctx.externalAxis.lower, ctx.externalAxis.upper);
			applyExternalAxisToTargetPos(ctx.externalAxis.axis, qExt, linkTarget.pos);
		}
		q = solveUrdfNumericalIk(ctx.urdfPath, ctx.ikLinkName, linkTarget, ctx.seedJointRad, maxIters, &failReason);
	}
	if (q.empty())
	{
		out.error = failReason.empty() ? std::string("IK未收敛/超迭代") : failReason;
		out.externalAxisQ = qExt;
		return out;
	}
	TeachIkContext residualCtx = ctx;
	residualCtx.externalAxis.qExternal = qExt;
	out.residualTcpMm = residualToolOriginMm(residualCtx, q);
	out.jointRad = std::move(q);
	out.externalAxisQ = qExt;
	out.ok = true;
	return out;
}

TeachIkResult solveTeachIkCoordinatedDrag(const TeachIkContext& ctxIn, const double qExternalHintMm,
										  const bool hasExternalHint)
{
	TeachIkResult out;
	if (!ctxIn.externalAxis.enabled)
	{
		return solveTeachIk(ctxIn);
	}

	const double qCurrent = std::clamp(ctxIn.externalAxis.qExternal, ctxIn.externalAxis.lower, ctxIn.externalAxis.upper);
	const double qHint =
		hasExternalHint ? std::clamp(qExternalHintMm, ctxIn.externalAxis.lower, ctxIn.externalAxis.upper) : qCurrent;
	const double hintDelta = std::abs(qHint - qCurrent);

	auto scoreCandidate = [&](const TeachIkResult& r) -> double {
		if (!r.ok || r.jointRad.size() != ctxIn.seedJointRad.size())
		{
			return 1e30;
		}
		double jointDelta = 0.0;
		for (size_t i = 0; i < r.jointRad.size(); ++i)
		{
			jointDelta += std::abs(r.jointRad[i] - ctxIn.seedJointRad[i]);
		}
		const double dQe = std::abs(r.externalAxisQ - qCurrent);
		return r.residualTcpMm + 12.0 * jointDelta + 0.012 * dQe;
	};

	TeachIkResult best;
	double bestScore = 1e30;
	auto consider = [&](TeachIkResult&& cand) {
		const double s = scoreCandidate(cand);
		if (s < bestScore)
		{
			bestScore = s;
			best = std::move(cand);
		}
	};

	// A：固定当前外轴（小步默认）
	{
		TeachIkContext a = ctxIn;
		a.externalAxis.qExternal = qCurrent;
		a.externalAxis.optimizeExternal = false;
		a.maxIkIterations = 12;
		consider(solveTeachIk(a));
	}
	// 臂已够好且无大沿轨提示 → 不再跑联立，拖动更跟手
	if (best.ok && best.residualTcpMm < 2.0 && hintDelta < 8.0)
	{
		return best;
	}

	// B：自适应联立
	{
		TeachIkContext b = ctxIn;
		b.externalAxis.qExternal = qCurrent;
		b.externalAxis.optimizeExternal = true;
		b.externalAxis.adaptiveExternalDamping = true;
		b.externalAxis.externalDeltaPriorWeight = 0.08;
		b.maxIkIterations = 16;
		consider(solveTeachIk(b));
	}
	if (best.ok && best.residualTcpMm < 1.5)
	{
		return best;
	}

	// C：大沿轨提示时再试投影种子
	if (hasExternalHint && hintDelta > 0.5)
	{
		TeachIkContext c = ctxIn;
		c.externalAxis.qExternal = qHint;
		c.externalAxis.optimizeExternal = true;
		c.externalAxis.adaptiveExternalDamping = true;
		c.externalAxis.externalDeltaPriorWeight = 0.02;
		c.maxIkIterations = 16;
		consider(solveTeachIk(c));
	}

	if (!best.ok)
	{
		out.error = "联立拖动无可行解";
		out.externalAxisQ = qCurrent;
		return out;
	}
	return best;
}
} // namespace RobotTeachIk
