/// @file RobotTeachIk.cpp
/// @brief RobotTeachIk 实现

#include "RobotTeachIk.h"

#include "RobotCoordinateFrames.h"
#include "RobotMatrixOsgBridge.h"

#include <QHash>
#include <QString>
#include <QVector>
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

	const double lambda = 1e-2;
	const int taskDim = useOrientation ? 6 : 3;
	const int iterLimit = maxIters > 0 ? maxIters : 180;
	const double orientationWeight = useOrientation ? 300.0 : 1.0;
	const int n = static_cast<int>(q.size());
	std::vector<double> J(static_cast<size_t>(taskDim * std::max(n, 1)), 0.0);
	std::vector<double> jtj(static_cast<size_t>(std::max(n, 1) * std::max(n, 1)), 0.0);
	std::vector<double> jte(static_cast<size_t>(std::max(n, 1)), 0.0);
	std::vector<double> e(static_cast<size_t>(taskDim), 0.0);
	QVector<double> qQt(n);
	for (int iter = 0; iter < iterLimit; ++iter)
	{
		for (int j = 0; j < n; ++j)
		{
			qQt[j] = q[static_cast<size_t>(j)];
		}
		double quatXyZw[4] = {0.0, 0.0, 0.0, 1.0};
		if (!UrdfRobotLoader::computeLinkPoseAndGeometricJacobian(
				urdfPath, qQt, ikLink, pos, useOrientation ? quatXyZw : nullptr, J, useOrientation, orientationWeight,
				nullptr) ||
			static_cast<int>(J.size()) < taskDim * n)
		{
			if (failReason)
			{
				*failReason = "无DH上下文";
			}
			return {};
		}
		if (useOrientation)
		{
			curQuat.set(quatXyZw[0], quatXyZw[1], quatXyZw[2], quatXyZw[3]);
			normalizeQuatSafe(curQuat);
		}
		e.assign(static_cast<size_t>(taskDim), 0.0);
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

		jtj.assign(static_cast<size_t>(n * n), 0.0);
		jte.assign(static_cast<size_t>(n), 0.0);
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
		const int nPos = static_cast<int>(qPos.size());
		std::vector<double> Jpos(static_cast<size_t>(3 * std::max(nPos, 1)), 0.0);
		std::vector<double> jtjPos(static_cast<size_t>(std::max(nPos, 1) * std::max(nPos, 1)), 0.0);
		std::vector<double> jtePos(static_cast<size_t>(std::max(nPos, 1)), 0.0);
		QVector<double> qPosQt(nPos);
		for (int iter = 0; iter < iterLimit; ++iter)
		{
			for (int j = 0; j < nPos; ++j)
			{
				qPosQt[j] = qPos[static_cast<size_t>(j)];
			}
			if (!UrdfRobotLoader::computeLinkPoseAndGeometricJacobian(urdfPath, qPosQt, ikLink, pos, nullptr, Jpos,
																	 false, 1.0, nullptr) ||
				static_cast<int>(Jpos.size()) < 3 * nPos)
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
			jtjPos.assign(static_cast<size_t>(nPos * nPos), 0.0);
			jtePos.assign(static_cast<size_t>(nPos), 0.0);
			const double ePos[3] = {e0, e1, e2};
			for (int r = 0; r < 3; ++r)
			{
				for (int c = 0; c < nPos; ++c)
				{
					jtePos[static_cast<size_t>(c)] += Jpos[static_cast<size_t>(r * nPos + c)] * ePos[r];
				}
			}
			for (int r = 0; r < nPos; ++r)
			{
				for (int c = 0; c < nPos; ++c)
				{
					double s = 0.0;
					for (int k = 0; k < 3; ++k)
					{
						s += Jpos[static_cast<size_t>(k * nPos + r)] * Jpos[static_cast<size_t>(k * nPos + c)];
					}
					jtjPos[static_cast<size_t>(r * nPos + c)] = s;
				}
			}
			for (int i = 0; i < nPos; ++i)
			{
				jtjPos[static_cast<size_t>(i * nPos + i)] += lambda * lambda;
			}
			if (!solveLinearSystem(jtjPos, jtePos, nPos))
			{
				break;
			}
			for (int j = 0; j < nPos; ++j)
			{
				jtePos[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jtePos[static_cast<size_t>(j)]));
				qPos[static_cast<size_t>(j)] += jtePos[static_cast<size_t>(j)];
			}
		}
	}

	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

RobotTeachIk::TeachIkExternalAxisDof dofFromLegacy(const RobotTeachIk::TeachIkExternalAxis& ax)
{
	RobotTeachIk::TeachIkExternalAxisDof dof;
	if (!ax.enabled)
	{
		return dof;
	}
	RobotTeachIk::TeachIkExternalAxisSlot slot;
	slot.configIndex = 0;
	slot.isPrismatic = ax.isPrismatic;
	slot.axis[0] = ax.axis[0];
	slot.axis[1] = ax.axis[1];
	slot.axis[2] = ax.axis[2];
	slot.originMm[0] = ax.originMm[0];
	slot.originMm[1] = ax.originMm[1];
	slot.originMm[2] = ax.originMm[2];
	slot.lower = ax.lower;
	slot.upper = ax.upper;
	dof.axes.push_back(slot);
	dof.qExternal.push_back(ax.qExternal);
	dof.optimizeExternal = ax.optimizeExternal;
	dof.externalDeltaPriorWeight = ax.externalDeltaPriorWeight;
	dof.adaptiveExternalDamping = ax.adaptiveExternalDamping;
	return dof;
}

RobotTeachIk::TeachIkExternalAxisDof resolveDof(const RobotTeachIk::TeachIkContext& ctx)
{
	if (ctx.externalAxes.active())
	{
		return ctx.externalAxes;
	}
	return dofFromLegacy(ctx.externalAxis);
}

void clampDofQ(RobotTeachIk::TeachIkExternalAxisDof& dof)
{
	for (size_t i = 0; i < dof.axes.size() && i < dof.qExternal.size(); ++i)
	{
		dof.qExternal[i] = std::clamp(dof.qExternal[i], dof.axes[i].lower, dof.axes[i].upper);
	}
}

bool dofAllPrismatic(const RobotTeachIk::TeachIkExternalAxisDof& dof)
{
	for (const auto& a : dof.axes)
	{
		if (!a.isPrismatic)
		{
			return false;
		}
	}
	return true;
}

void rotatePointAboutAxis(const double origin[3], const double axis[3], const double angleRad, double p[3])
{
	const double c = std::cos(angleRad);
	const double s = std::sin(angleRad);
	const double t = 1.0 - c;
	const double ax = axis[0];
	const double ay = axis[1];
	const double az = axis[2];
	const double px = p[0] - origin[0];
	const double py = p[1] - origin[1];
	const double pz = p[2] - origin[2];
	const double r00 = t * ax * ax + c;
	const double r01 = t * ax * ay - s * az;
	const double r02 = t * ax * az + s * ay;
	const double r10 = t * ax * ay + s * az;
	const double r11 = t * ay * ay + c;
	const double r12 = t * ay * az - s * ax;
	const double r20 = t * ax * az - s * ay;
	const double r21 = t * ay * az + s * ax;
	const double r22 = t * az * az + c;
	p[0] = origin[0] + r00 * px + r01 * py + r02 * pz;
	p[1] = origin[1] + r10 * px + r11 * py + r12 * pz;
	p[2] = origin[2] + r20 * px + r21 * py + r22 * pz;
}

void applyOneAxisUnbake(const RobotTeachIk::TeachIkExternalAxisSlot& slot, const double q, double pos[3],
						osg::Quat* quat)
{
	if (slot.isPrismatic)
	{
		pos[0] -= q * slot.axis[0];
		pos[1] -= q * slot.axis[1];
		pos[2] -= q * slot.axis[2];
		return;
	}
	// 与 compose 互逆：绕 origin 取 -q
	rotatePointAboutAxis(slot.originMm, slot.axis, -q, pos);
	if (quat)
	{
		osg::Quat r;
		r.makeRotate(-q, osg::Vec3d(slot.axis[0], slot.axis[1], slot.axis[2]));
		*quat = r * (*quat);
		normalizeQuatSafe(*quat);
	}
}

void applyDofUnbakeToTarget(const RobotTeachIk::TeachIkExternalAxisDof& dof, IkLinkTarget& target)
{
	if (!dof.active())
	{
		return;
	}
	// 逆序消轴，对齐 unbakeAxesChain
	for (int i = static_cast<int>(dof.axes.size()) - 1; i >= 0; --i)
	{
		applyOneAxisUnbake(dof.axes[static_cast<size_t>(i)], dof.qExternal[static_cast<size_t>(i)], target.pos,
						   target.hasOrientation ? &target.quat : nullptr);
	}
}

void fillResultExternalQs(RobotTeachIk::TeachIkResult& out, const RobotTeachIk::TeachIkExternalAxisDof& dof,
						  const int configCount)
{
	out.externalAxisQ = dof.qExternal.empty() ? 0.0 : dof.qExternal.front();
	if (configCount > 0)
	{
		out.externalAxisQs.assign(static_cast<size_t>(configCount), 0.0);
		for (size_t i = 0; i < dof.axes.size() && i < dof.qExternal.size(); ++i)
		{
			const int idx = dof.axes[i].configIndex;
			if (idx >= 0 && idx < configCount)
			{
				out.externalAxisQs[static_cast<size_t>(idx)] = dof.qExternal[i];
			}
		}
		return;
	}
	out.externalAxisQs = dof.qExternal;
}

/// 全平移且 DOF<=2：外轴并入 DLS；含旋转或更高维走网格+固定臂 IK
std::vector<double> solveUrdfNumericalIkCoupledExternalMulti(
	const QString& urdfPath, const QString& ikLink, const IkLinkTarget& linkTargetWorld, std::vector<double> q,
	RobotTeachIk::TeachIkExternalAxisDof& dofInOut, const int maxIters, std::string* failReason)
{
	if (!dofInOut.active() || !dofAllPrismatic(dofInOut) || dofInOut.axes.size() > 2)
	{
		if (failReason)
		{
			*failReason = "联立外轴仅支持≤2 平移轴";
		}
		return {};
	}
	if (urdfPath.isEmpty() || ikLink.isEmpty() || q.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}

	const double target[3] = {linkTargetWorld.pos[0], linkTargetWorld.pos[1], linkTargetWorld.pos[2]};
	const bool useOrientation = linkTargetWorld.hasOrientation;
	const osg::Quat targetQuat = useOrientation ? linkTargetWorld.quat : osg::Quat();
	const double lambda = 1e-2;
	const int taskDim = useOrientation ? 6 : 3;
	const int iterLimit = maxIters > 0 ? maxIters : 180;
	const double orientationWeight = useOrientation ? 300.0 : 1.0;
	const int nArm = static_cast<int>(q.size());
	const int nExt = static_cast<int>(dofInOut.axes.size());
	const int n = nArm + nExt;
	clampDofQ(dofInOut);
	std::vector<double> qeSeed = dofInOut.qExternal;

	std::vector<double> bestQ = q;
	std::vector<double> bestQe = dofInOut.qExternal;
	double bestPosErr = 1e30;
	std::vector<double> Jarm;
	QVector<double> qQt(nArm);

	for (int iter = 0; iter < iterLimit; ++iter)
	{
		double pos[3] = {0.0, 0.0, 0.0};
		osg::Quat curQuat;
		for (int j = 0; j < nArm; ++j)
		{
			qQt[j] = q[static_cast<size_t>(j)];
		}
		double quatXyZw[4] = {0.0, 0.0, 0.0, 1.0};
		if (!UrdfRobotLoader::computeLinkPoseAndGeometricJacobian(
				urdfPath, qQt, ikLink, pos, useOrientation ? quatXyZw : nullptr, Jarm, useOrientation,
				orientationWeight, nullptr) ||
			static_cast<int>(Jarm.size()) < taskDim * nArm)
		{
			if (failReason)
			{
				*failReason = "无DH上下文";
			}
			return {};
		}
		if (useOrientation)
		{
			curQuat.set(quatXyZw[0], quatXyZw[1], quatXyZw[2], quatXyZw[3]);
			normalizeQuatSafe(curQuat);
		}
		double pEff[3] = {pos[0], pos[1], pos[2]};
		for (int eIdx = 0; eIdx < nExt; ++eIdx)
		{
			const auto& ax = dofInOut.axes[static_cast<size_t>(eIdx)];
			const double qe = dofInOut.qExternal[static_cast<size_t>(eIdx)];
			pEff[0] += qe * ax.axis[0];
			pEff[1] += qe * ax.axis[1];
			pEff[2] += qe * ax.axis[2];
		}
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
			bestQe = dofInOut.qExternal;
		}
		if (posErr < 1e-2 && (!useOrientation || rotErr < 0.1 * kDegToRad))
		{
			dofInOut.qExternal = bestQe;
			return bestQ;
		}

		std::vector<double> J(static_cast<size_t>(taskDim * n), 0.0);
		for (int j = 0; j < nArm; ++j)
		{
			J[0 * n + j] = Jarm[static_cast<size_t>(0 * nArm + j)];
			J[1 * n + j] = Jarm[static_cast<size_t>(1 * nArm + j)];
			J[2 * n + j] = Jarm[static_cast<size_t>(2 * nArm + j)];
			if (useOrientation)
			{
				J[3 * n + j] = Jarm[static_cast<size_t>(3 * nArm + j)];
				J[4 * n + j] = Jarm[static_cast<size_t>(4 * nArm + j)];
				J[5 * n + j] = Jarm[static_cast<size_t>(5 * nArm + j)];
			}
		}
		for (int ei = 0; ei < nExt; ++ei)
		{
			const auto& ax = dofInOut.axes[static_cast<size_t>(ei)];
			J[0 * n + nArm + ei] = ax.axis[0];
			J[1 * n + nArm + ei] = ax.axis[1];
			J[2 * n + nArm + ei] = ax.axis[2];
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
		for (int ei = 0; ei < nExt; ++ei)
		{
			double lambdaExtScale = 1.0;
			if (dofInOut.adaptiveExternalDamping && posErr > 1e-6)
			{
				const auto& ax = dofInOut.axes[static_cast<size_t>(ei)];
				const double ePar = e[0] * ax.axis[0] + e[1] * ax.axis[1] + e[2] * ax.axis[2];
				const double railShare = std::min(1.0, std::abs(ePar) / posErr);
				double armAlong = 0.0;
				for (int j = 0; j < nArm; ++j)
				{
					const double jPar =
						J[0 * n + j] * ax.axis[0] + J[1 * n + j] * ax.axis[1] + J[2 * n + j] * ax.axis[2];
					armAlong += jPar * jPar;
				}
				armAlong = std::sqrt(armAlong);
				const double armWeak = 1.0 / (1.0 + armAlong);
				lambdaExtScale = 0.25 + 2.5 * (1.0 - railShare) + 1.5 * (1.0 - armWeak) * (1.0 - railShare);
			}
			jtj[static_cast<size_t>((nArm + ei) * n + (nArm + ei))] +=
				(lambda * lambdaExtScale) * (lambda * lambdaExtScale);
			if (dofInOut.externalDeltaPriorWeight > 0.0)
			{
				jtj[static_cast<size_t>((nArm + ei) * n + (nArm + ei))] += dofInOut.externalDeltaPriorWeight;
				jte[static_cast<size_t>(nArm + ei)] -=
					dofInOut.externalDeltaPriorWeight *
					(dofInOut.qExternal[static_cast<size_t>(ei)] - qeSeed[static_cast<size_t>(ei)]);
			}
		}
		if (!solveLinearSystem(jtj, jte, n))
		{
			if (failReason)
			{
				*failReason = "IK未收敛/超迭代";
			}
			if (bestPosErr < 1e29)
			{
				dofInOut.qExternal = bestQe;
				return bestQ;
			}
			return {};
		}
		for (int j = 0; j < nArm; ++j)
		{
			jte[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, jte[static_cast<size_t>(j)]));
			q[static_cast<size_t>(j)] += jte[static_cast<size_t>(j)];
		}
		for (int e = 0; e < nExt; ++e)
		{
			const double stepCap = dofInOut.axes[static_cast<size_t>(e)].isPrismatic ? 50.0 : 0.2;
			const double dqExt =
				std::max(-stepCap, std::min(stepCap, jte[static_cast<size_t>(nArm + e)]));
			dofInOut.qExternal[static_cast<size_t>(e)] = std::clamp(
				dofInOut.qExternal[static_cast<size_t>(e)] + dqExt, dofInOut.axes[static_cast<size_t>(e)].lower,
				dofInOut.axes[static_cast<size_t>(e)].upper);
		}
	}

	if (bestPosErr < 1e29)
	{
		dofInOut.qExternal = bestQe;
		return bestQ;
	}
	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

int gridPointsForSpan(const double span, const int dofCount)
{
	if (span < 1e-9)
	{
		return 1;
	}
	if (dofCount <= 1)
	{
		return std::clamp(static_cast<int>(span / 200.0) + 1, 3, 7);
	}
	if (dofCount == 2)
	{
		return std::clamp(static_cast<int>(span / 250.0) + 1, 3, 5);
	}
	return std::clamp(static_cast<int>(span / 300.0) + 1, 2, 3);
}

void appendGridSamples(const RobotTeachIk::TeachIkExternalAxisDof& seedDof, std::vector<std::vector<double>>& outSamples)
{
	const int dofN = static_cast<int>(seedDof.axes.size());
	if (dofN <= 0)
	{
		return;
	}
	std::vector<int> gridN(static_cast<size_t>(dofN), 1);
	int total = 1;
	for (int i = 0; i < dofN; ++i)
	{
		const double span = std::max(0.0, seedDof.axes[static_cast<size_t>(i)].upper - seedDof.axes[static_cast<size_t>(i)].lower);
		gridN[static_cast<size_t>(i)] = gridPointsForSpan(span, dofN);
		total *= gridN[static_cast<size_t>(i)];
	}
	// 防组合爆炸
	while (total > 64 && dofN > 0)
	{
		int maxIdx = 0;
		for (int i = 1; i < dofN; ++i)
		{
			if (gridN[static_cast<size_t>(i)] > gridN[static_cast<size_t>(maxIdx)])
			{
				maxIdx = i;
			}
		}
		if (gridN[static_cast<size_t>(maxIdx)] <= 2)
		{
			break;
		}
		total /= gridN[static_cast<size_t>(maxIdx)];
		--gridN[static_cast<size_t>(maxIdx)];
		total *= gridN[static_cast<size_t>(maxIdx)];
	}

	std::vector<int> idx(static_cast<size_t>(dofN), 0);
	for (;;)
	{
		std::vector<double> sample(static_cast<size_t>(dofN), 0.0);
		for (int i = 0; i < dofN; ++i)
		{
			const auto& ax = seedDof.axes[static_cast<size_t>(i)];
			const int gn = gridN[static_cast<size_t>(i)];
			const double t = gn <= 1 ? 0.0 : static_cast<double>(idx[static_cast<size_t>(i)]) / static_cast<double>(gn - 1);
			sample[static_cast<size_t>(i)] = ax.lower + t * (ax.upper - ax.lower);
		}
		outSamples.push_back(std::move(sample));

		int carry = 0;
		++idx[0];
		while (carry < dofN && idx[static_cast<size_t>(carry)] >= gridN[static_cast<size_t>(carry)])
		{
			idx[static_cast<size_t>(carry)] = 0;
			++carry;
			if (carry < dofN)
			{
				++idx[static_cast<size_t>(carry)];
			}
		}
		if (carry >= dofN)
		{
			break;
		}
	}
}

double residualToolOriginMm(const RobotTeachIk::TeachIkContext& ctx, const std::vector<double>& q,
							const RobotTeachIk::TeachIkExternalAxisDof& dof)
{
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromTeachContext(ctx, linkTarget))
	{
		return 1e30;
	}
	double toolOriginPos[3]{};
	ctx.T_base_target.translationMm(toolOriginPos[0], toolOriginPos[1], toolOriginPos[2]);
	osg::Quat toolQuat = linkTarget.quat;
	if (dof.active())
	{
		RobotTeachIk::TeachIkExternalAxisDof tmp = dof;
		IkLinkTarget t{};
		t.pos[0] = toolOriginPos[0];
		t.pos[1] = toolOriginPos[1];
		t.pos[2] = toolOriginPos[2];
		t.quat = toolQuat;
		t.hasOrientation = linkTarget.hasOrientation;
		applyDofUnbakeToTarget(tmp, t);
		toolOriginPos[0] = t.pos[0];
		toolOriginPos[1] = t.pos[1];
		toolOriginPos[2] = t.pos[2];
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

RobotTeachIk::TeachIkResult solveFixedExternalThenArm(const RobotTeachIk::TeachIkContext& ctx,
													  RobotTeachIk::TeachIkExternalAxisDof dof, const int maxIters,
													  std::string* failReason)
{
	RobotTeachIk::TeachIkResult out;
	IkLinkTarget linkTarget{};
	if (!ikLinkTargetFromTeachContext(ctx, linkTarget))
	{
		out.error = "无效目标位姿";
		return out;
	}
	clampDofQ(dof);
	applyDofUnbakeToTarget(dof, linkTarget);
	std::vector<double> q =
		solveUrdfNumericalIk(ctx.urdfPath, ctx.ikLinkName, linkTarget, ctx.seedJointRad, maxIters, failReason);
	if (q.empty())
	{
		out.error = failReason && !failReason->empty() ? *failReason : std::string("IK未收敛/超迭代");
		fillResultExternalQs(out, dof, ctx.externalAxisConfigCount);
		return out;
	}
	out.residualTcpMm = residualToolOriginMm(ctx, q, dof);
	out.jointRad = std::move(q);
	fillResultExternalQs(out, dof, ctx.externalAxisConfigCount);
	out.ok = true;
	return out;
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

void applyExternalAxesToTarget(const TeachIkExternalAxisDof& dof, double posInOut[3], double quatXyzwInOut[4])
{
	if (!posInOut || !dof.active())
	{
		return;
	}
	osg::Quat quat(0.0, 0.0, 0.0, 1.0);
	const bool hasQuat = quatXyzwInOut != nullptr;
	if (hasQuat)
	{
		quat.set(quatXyzwInOut[0], quatXyzwInOut[1], quatXyzwInOut[2], quatXyzwInOut[3]);
		normalizeQuatSafe(quat);
	}
	for (int i = static_cast<int>(dof.axes.size()) - 1; i >= 0; --i)
	{
		applyOneAxisUnbake(dof.axes[static_cast<size_t>(i)], dof.qExternal[static_cast<size_t>(i)], posInOut,
						   hasQuat ? &quat : nullptr);
	}
	if (hasQuat)
	{
		quatXyzwInOut[0] = quat.x();
		quatXyzwInOut[1] = quat.y();
		quatXyzwInOut[2] = quat.z();
		quatXyzwInOut[3] = quat.w();
	}
}

TeachIkResult solveTeachIk(const TeachIkContext& ctx)
{
	TeachIkResult out;
	if (ctx.urdfPath.isEmpty() || ctx.ikLinkName.isEmpty() || ctx.seedJointRad.empty())
	{
		out.error = "无DH上下文";
		return out;
	}

	TeachIkExternalAxisDof dof = resolveDof(ctx);
	const int maxIters = ctx.maxIkIterations > 0 ? ctx.maxIkIterations : 180;
	if (!dof.active())
	{
		IkLinkTarget linkTarget{};
		if (!ikLinkTargetFromTeachContext(ctx, linkTarget))
		{
			out.error = "无效目标位姿";
			return out;
		}
		std::string failReason;
		std::vector<double> q =
			solveUrdfNumericalIk(ctx.urdfPath, ctx.ikLinkName, linkTarget, ctx.seedJointRad, maxIters, &failReason);
		if (q.empty())
		{
			out.error = failReason.empty() ? std::string("IK未收敛/超迭代") : failReason;
			return out;
		}
		out.residualTcpMm = residualToolOriginMm(ctx, q, dof);
		out.jointRad = std::move(q);
		out.ok = true;
		return out;
	}

	clampDofQ(dof);
	const int dofN = static_cast<int>(dof.axes.size());
	std::string failReason;

	// DOF<=2 且全平移：可走联立 DLS
	if (dof.optimizeExternal && dofN <= 2 && dofAllPrismatic(dof))
	{
		IkLinkTarget linkTargetWorld{};
		if (!ikLinkTargetFromTeachContext(ctx, linkTargetWorld))
		{
			out.error = "无效目标位姿";
			return out;
		}
		std::vector<double> q = solveUrdfNumericalIkCoupledExternalMulti(ctx.urdfPath, ctx.ikLinkName, linkTargetWorld,
																		 ctx.seedJointRad, dof, maxIters, &failReason);
		if (q.empty())
		{
			out.error = failReason.empty() ? std::string("IK未收敛/超迭代") : failReason;
			fillResultExternalQs(out, dof, ctx.externalAxisConfigCount);
			return out;
		}
		out.residualTcpMm = residualToolOriginMm(ctx, q, dof);
		out.jointRad = std::move(q);
		fillResultExternalQs(out, dof, ctx.externalAxisConfigCount);
		out.ok = true;
		return out;
	}

	// 固定外轴：直接 unbake + 臂 IK
	if (!dof.optimizeExternal)
	{
		return solveFixedExternalThenArm(ctx, dof, maxIters, &failReason);
	}

	// optimizeExternal 且（DOF>2 或含旋转）：网格采样外轴再臂 IK
	std::vector<std::vector<double>> samples;
	samples.push_back(dof.qExternal);
	appendGridSamples(dof, samples);

	TeachIkResult best;
	double bestRes = 1e30;
	for (const auto& sample : samples)
	{
		TeachIkExternalAxisDof tryDof = dof;
		tryDof.qExternal = sample;
		tryDof.optimizeExternal = false;
		TeachIkContext tryCtx = ctx;
		tryCtx.externalAxes = tryDof;
		tryCtx.externalAxis.enabled = false;
		if (best.ok && !best.jointRad.empty())
		{
			tryCtx.seedJointRad = best.jointRad;
		}
		const TeachIkResult r = solveFixedExternalThenArm(tryCtx, tryDof, maxIters, &failReason);
		if (!r.ok)
		{
			continue;
		}
		if (r.residualTcpMm < bestRes)
		{
			bestRes = r.residualTcpMm;
			best = r;
			dof.qExternal = sample;
		}
		if (bestRes < 1.0)
		{
			break;
		}
	}

	if (!best.ok)
	{
		out.error = failReason.empty() ? std::string("外轴网格未找到可行解") : failReason;
		fillResultExternalQs(out, dof, ctx.externalAxisConfigCount);
		return out;
	}

	// DOF<=2 全平移：网格优解后再联立细化
	if (dofN <= 2 && dofAllPrismatic(dof))
	{
		TeachIkExternalAxisDof refineDof = dof;
		refineDof.optimizeExternal = true;
		refineDof.adaptiveExternalDamping = true;
		refineDof.externalDeltaPriorWeight = 0.03;
		IkLinkTarget linkTargetWorld{};
		if (ikLinkTargetFromTeachContext(ctx, linkTargetWorld))
		{
			std::vector<double> seed = best.jointRad.empty() ? ctx.seedJointRad : best.jointRad;
			std::vector<double> q = solveUrdfNumericalIkCoupledExternalMulti(
				ctx.urdfPath, ctx.ikLinkName, linkTargetWorld, seed, refineDof, std::max(maxIters, 56), &failReason);
			if (!q.empty())
			{
				best.jointRad = std::move(q);
				best.residualTcpMm = residualToolOriginMm(ctx, best.jointRad, refineDof);
				fillResultExternalQs(best, refineDof, ctx.externalAxisConfigCount);
				best.ok = true;
			}
		}
	}

	return best;
}

TeachIkResult solveTeachIkCoordinatedDrag(const TeachIkContext& ctxIn, const double qExternalHintMm,
										  const bool hasExternalHint)
{
	TeachIkExternalAxisDof dof = resolveDof(ctxIn);
	if (!dof.active())
	{
		return solveTeachIk(ctxIn);
	}

	// 多轴：先试固定外轴，再联立/采样；避免只拧臂
	if (dof.axes.size() > 1)
	{
		TeachIkContext ctx = ctxIn;
		ctx.externalAxes = dof;
		ctx.externalAxis.enabled = false;
		ctx.externalAxes.adaptiveExternalDamping = true;
		ctx.externalAxes.externalDeltaPriorWeight = 0.05;
		ctx.maxIkIterations = ctx.maxIkIterations > 0 ? ctx.maxIkIterations : 16;

		auto score = [&](const TeachIkResult& r) -> double {
			if (!r.ok || r.jointRad.size() != ctxIn.seedJointRad.size())
			{
				return 1e30;
			}
			double jointDelta = 0.0;
			for (size_t i = 0; i < r.jointRad.size(); ++i)
			{
				jointDelta += std::abs(r.jointRad[i] - ctxIn.seedJointRad[i]);
			}
			double dQe = 0.0;
			for (size_t i = 0; i < dof.axes.size(); ++i)
			{
				const int cidx = dof.axes[i].configIndex;
				double qNew = dof.qExternal[i];
				if (ctxIn.externalAxisConfigCount > 0 && cidx >= 0 &&
					cidx < static_cast<int>(r.externalAxisQs.size()))
				{
					qNew = r.externalAxisQs[static_cast<size_t>(cidx)];
				}
				else if (i < r.externalAxisQs.size())
				{
					qNew = r.externalAxisQs[i];
				}
				dQe += std::abs(qNew - dof.qExternal[i]);
			}
			return r.residualTcpMm + 12.0 * jointDelta + 0.012 * dQe;
		};

		TeachIkContext fixedCtx = ctx;
		fixedCtx.externalAxes.optimizeExternal = false;
		fixedCtx.maxIkIterations = 12;
		TeachIkResult fixedTry = solveTeachIk(fixedCtx);

		TeachIkContext optCtx = ctx;
		optCtx.externalAxes.optimizeExternal = true;
		TeachIkResult optimized = solveTeachIk(optCtx);

		const double sFixed = score(fixedTry);
		const double sOpt = score(optimized);
		if (sOpt <= sFixed)
		{
			return optimized.ok ? optimized : fixedTry;
		}
		return fixedTry.ok ? fixedTry : optimized;
	}

	const double qCurrent = std::clamp(dof.qExternal.front(), dof.axes.front().lower, dof.axes.front().upper);
	const double qHint =
		hasExternalHint ? std::clamp(qExternalHintMm, dof.axes.front().lower, dof.axes.front().upper) : qCurrent;
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

	auto makeSingleCtx = [&](const double qe, const bool optimize) {
		TeachIkContext a = ctxIn;
		if (a.externalAxes.active())
		{
			a.externalAxes.qExternal[0] = qe;
			a.externalAxes.optimizeExternal = optimize;
			a.externalAxes.adaptiveExternalDamping = true;
			a.externalAxis.enabled = false;
		}
		else
		{
			a.externalAxis.qExternal = qe;
			a.externalAxis.optimizeExternal = optimize;
			a.externalAxis.adaptiveExternalDamping = true;
		}
		return a;
	};

	// A：固定当前外轴（小步默认）
	{
		TeachIkContext a = makeSingleCtx(qCurrent, false);
		a.maxIkIterations = 12;
		consider(solveTeachIk(a));
	}
	if (best.ok && best.residualTcpMm < 2.0 && hintDelta < 8.0)
	{
		return best;
	}

	// B：自适应联立
	{
		TeachIkContext b = makeSingleCtx(qCurrent, true);
		if (b.externalAxes.active())
		{
			b.externalAxes.externalDeltaPriorWeight = 0.08;
		}
		else
		{
			b.externalAxis.externalDeltaPriorWeight = 0.08;
		}
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
		TeachIkContext c = makeSingleCtx(qHint, true);
		if (c.externalAxes.active())
		{
			c.externalAxes.externalDeltaPriorWeight = 0.02;
		}
		else
		{
			c.externalAxis.externalDeltaPriorWeight = 0.02;
		}
		c.maxIkIterations = 16;
		consider(solveTeachIk(c));
	}

	if (!best.ok)
	{
		TeachIkResult out;
		out.error = "联立拖动无可行解";
		out.externalAxisQ = qCurrent;
		out.externalAxisQs = {qCurrent};
		if (ctxIn.externalAxisConfigCount > 0)
		{
			out.externalAxisQs.assign(static_cast<size_t>(ctxIn.externalAxisConfigCount), 0.0);
			const int idx = dof.axes.front().configIndex;
			if (idx >= 0 && idx < ctxIn.externalAxisConfigCount)
			{
				out.externalAxisQs[static_cast<size_t>(idx)] = qCurrent;
			}
		}
		return out;
	}
	return best;
}
} // namespace RobotTeachIk
