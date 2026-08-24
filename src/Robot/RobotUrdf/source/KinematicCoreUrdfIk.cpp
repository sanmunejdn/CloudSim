#include "KinematicCoreUrdfIk.h"

#include "UrdfKinematicsWorkspace.h"
#include "UrdfRobotLoader.h"

#include "GeometricJacobian.h"
#include "KinematicGraph.h"

#include <QHash>
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

bool linkPoseFromUrdfGraph(const QString& urdfPath, const QString& linkName, const QVector<double>& qRad,
						   double outPos[3], osg::Quat* outQuat, QString* errorMessage)
{
	QHash<QString, osg::Matrixd> linkWorld;
	if (!computeLinkWorldMatrices(urdfPath, qRad, linkWorld, errorMessage) || !linkWorld.contains(linkName))
	{
		if (errorMessage && errorMessage->isEmpty())
		{
			*errorMessage = QStringLiteral("Link '%1' missing in FK.").arg(linkName);
		}
		return false;
	}
	const osg::Matrixd& m = linkWorld.value(linkName);
	const osg::Vec3d t = m.getTrans();
	outPos[0] = t.x();
	outPos[1] = t.y();
	outPos[2] = t.z();
	if (outQuat)
	{
		*outQuat = m.getRotate();
		normalizeQuatSafe(*outQuat);
	}
	return true;
}

std::vector<double> runUrdfDlsLoop(const QString& urdfPath, const QString& ikLink,
								   const kinematic_core::KinematicGraph& graph, const int linkIdx,
								   const UrdfPoseIkTarget& target, std::vector<double> q,
								   const UrdfIkSolverOptions& options, std::string* failReason)
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

	double base[16];
	for (int i = 0; i < 16; ++i)
	{
		base[i] = (i % 5 == 0) ? 1.0 : 0.0;
	}

	kinematic_core::JacobianOptions jopt;
	jopt.orientationWeight = orientationWeight;

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
		osg::Quat curQuat;
		if (!linkPoseFromUrdfGraph(urdfPath, ikLink, qRad, pos, useOrientation ? &curQuat : nullptr, nullptr))
		{
			if (failReason)
			{
				*failReason = "URDF FK failed";
			}
			break;
		}

		if (useOrientation)
		{
			if (!kinematic_core::computePoseJacobian(graph, base, qRad.constData(),
													 static_cast<std::size_t>(qRad.size()), linkIdx, ws.J, jopt) ||
				static_cast<int>(ws.J.size()) < taskDim * n)
			{
				if (failReason)
				{
					*failReason = "URDF pose Jacobian failed";
				}
				break;
			}
		}
		else if (!kinematic_core::computePositionJacobian(graph, base, qRad.constData(),
														  static_cast<std::size_t>(qRad.size()), linkIdx, ws.J, jopt) ||
				 static_cast<int>(ws.J.size()) < taskDim * n)
		{
			if (failReason)
			{
				*failReason = "URDF position Jacobian failed";
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

		if (posErr < bestPosErr)
		{
			bestPosErr = posErr;
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

	if (bestPosErr <= posTol * 12.0)
	{
		return bestQ;
	}

	if (useOrientation)
	{
		UrdfPoseIkTarget posOnly = target;
		posOnly.hasOrientation = false;
		UrdfIkSolverOptions posOpt = options;
		posOpt.maxIterations = std::max(iterLimit, 24);
		return runUrdfDlsLoop(urdfPath, ikLink, graph, linkIdx, posOnly, bestQ, posOpt, failReason);
	}

	if (failReason)
	{
		*failReason = "KinematicCore DLS did not converge";
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
	if (!linkPoseFromUrdfGraph(urdfPath, linkName, jointAnglesRad, outPosMm, outQuatXyzw ? &quat : nullptr,
							   errorMessage))
	{
		return false;
	}
	if (outQuatXyzw)
	{
		outQuatXyzw[0] = quat.x();
		outQuatXyzw[1] = quat.y();
		outQuatXyzw[2] = quat.z();
		outQuatXyzw[3] = quat.w();
	}

	double base[16];
	for (int i = 0; i < 16; ++i)
	{
		base[i] = (i % 5 == 0) ? 1.0 : 0.0;
	}
	kinematic_core::JacobianOptions jopt;
	jopt.orientationWeight = orientationWeight;
	if (includeOrientation)
	{
		if (!kinematic_core::computePoseJacobian(graph, base, jointAnglesRad.constData(),
												 static_cast<std::size_t>(jointAnglesRad.size()), linkIdx,
												 outJ_rowMajor, jopt))
		{
			return false;
		}
		return static_cast<int>(outJ_rowMajor.size()) >= 6 * n;
	}
	if (!kinematic_core::computePositionJacobian(graph, base, jointAnglesRad.constData(),
												 static_cast<std::size_t>(jointAnglesRad.size()), linkIdx,
												 outJ_rowMajor, jopt))
	{
		return false;
	}
	return static_cast<int>(outJ_rowMajor.size()) >= 3 * n;
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
