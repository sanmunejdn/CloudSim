#include "RobotInstructionController.h"
#include "RunLogger.h"
#include "UrdfRobotLoader.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <QHash>
#include <QString>
#include <QStringList>

#include <osg/Matrixd>
#include <osg/Quat>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

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

osg::Quat eulerDegToQuat(const RobotInstruction::Vec3& eulerDeg)
{
	const double rx = eulerDeg.x * kDegToRad;
	const double ry = eulerDeg.y * kDegToRad;
	const double rz = eulerDeg.z * kDegToRad;
	const double cx = std::cos(rx);
	const double sx = std::sin(rx);
	const double cy = std::cos(ry);
	const double sy = std::sin(ry);
	const double cz = std::cos(rz);
	const double sz = std::sin(rz);
	// Keep exactly the same convention as backendvisual_math::eulerDegToQuat:
	// R = Rz(rz) * Ry(ry) * Rx(rx), then extract quaternion from rotation matrix.
	osg::Matrixd m;
	m(0, 0) = cy * cz;
	m(0, 1) = cz * sx * sy - cx * sz;
	m(0, 2) = sx * sz + cx * cz * sy;
	m(1, 0) = cy * sz;
	m(1, 1) = cx * cz + sx * sy * sz;
	m(1, 2) = cx * sy * sz - cz * sx;
	m(2, 0) = -sy;
	m(2, 1) = cy * sx;
	m(2, 2) = cx * cy;
	m(0, 3) = 0.0;
	m(1, 3) = 0.0;
	m(2, 3) = 0.0;
	m(3, 0) = 0.0;
	m(3, 1) = 0.0;
	m(3, 2) = 0.0;
	m(3, 3) = 1.0;
	osg::Quat q = m.getRotate();
	normalizeQuatSafe(q);
	return q;
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

double trapezoidDuration(double distance, double vmax, double amax)
{
	const double d = std::max(0.0, distance);
	const double v = std::max(1e-6, vmax);
	const double a = std::max(1e-6, amax);
	if (d < 1e-9)
	{
		return 0.05;
	}
	const double tAcc = v / a;
	const double dAcc = 0.5 * a * tAcc * tAcc;
	double t = 0.0;
	if (d <= 2.0 * dAcc)
	{
		t = 2.0 * std::sqrt(d / a);
	}
	else
	{
		t = 2.0 * tAcc + (d - 2.0 * dAcc) / v;
	}
	return std::max(0.05, t);
}

std::vector<double> parseCsvDoubles(const std::string& text)
{
	std::vector<double> out;
	std::stringstream ss(text);
	std::string token;
	while (std::getline(ss, token, ','))
	{
		if (token.empty())
		{
			continue;
		}
		try
		{
			out.push_back(std::stod(token));
		}
		catch (...)
		{
			return {};
		}
	}
	return out;
}

std::vector<double> currentJointVectorFromInstruction(const RobotInstruction::Base& cmd)
{
	const auto& ext = cmd.extensionProperties();
	const auto it = ext.find("context.currentJointRadCsv");
	if (it == ext.end())
	{
		return {};
	}
	return parseCsvDoubles(it->second);
}

bool hasTcpLinkContext(const RobotInstruction::Base& cmd)
{
	const auto& ext = cmd.extensionProperties();
	const auto it = ext.find("context.tcpLinkName");
	return it != ext.end() && !it->second.empty();
}

bool legacyJointDeltaFromInstruction(const RobotInstruction::Base& cmd, int& outJointIndex, double& outDeltaRad)
{
	const auto& ext = cmd.extensionProperties();
	const auto itIdx = ext.find("legacy.jointIndex");
	const auto itDeg = ext.find("legacy.angleDeg");
	if (itIdx == ext.end() || itDeg == ext.end())
	{
		return false;
	}
	try
	{
		outJointIndex = std::stoi(itIdx->second);
		outDeltaRad = std::stod(itDeg->second) * kDegToRad;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

std::vector<double> solveTargetByLegacyJointDelta(const RobotInstruction::Base& cmd)
{
	std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		return {};
	}
	int jidx = -1;
	double dq = 0.0;
	if (!legacyJointDeltaFromInstruction(cmd, jidx, dq))
	{
		return {};
	}
	if (jidx < 0 || jidx >= static_cast<int>(q.size()))
	{
		return {};
	}
	q[static_cast<size_t>(jidx)] += dq;
	return q;
}

std::vector<double> solveTargetByIkIfPossible(
	const RobotInstruction::Base& cmd,
	const std::vector<robot_kinematics::DhRow>& dhRows,
	std::string* failReason)
{
	if (dhRows.empty() || !cmd.hasPoseProperty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const std::size_t nJoint = robot_kinematics::jointCountFromDhRows(dhRows);
	if (nJoint == 0 || q.size() != nJoint)
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const RobotInstruction::Vec3 p = cmd.pose();
	const double targetPos[3] = { p.x, p.y, p.z };

	double reachMm = 0.0;
	for (const robot_kinematics::DhRow& row : dhRows)
	{
		reachMm += std::sqrt(row.a * row.a + row.d * row.d);
	}
	reachMm = std::max(reachMm, 1.0);
	const double targetNormMm = std::sqrt(targetPos[0] * targetPos[0] + targetPos[1] * targetPos[1] + targetPos[2] * targetPos[2]);
	if (targetNormMm > reachMm * 2.5 || targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	double curPos[3] = { 0.0, 0.0, 0.0 };
	if (robot_kinematics::endEffectorPosition(dhRows, q, curPos))
	{
		const double dx = targetPos[0] - curPos[0];
		const double dy = targetPos[1] - curPos[1];
		const double dz = targetPos[2] - curPos[2];
		const double moveDistanceMm = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (moveDistanceMm > reachMm * 2.0)
		{
			if (failReason)
			{
				*failReason = "目标越界/单位不一致";
			}
			return {};
		}
	}

	std::vector<double> qSolved = q;
	int iters = 0;
	if (!robot_kinematics::ikPositionDampedLeastSquares(dhRows, targetPos, qSolved, 120, 1e-3, 1e-2, &iters))
	{
		if (failReason)
		{
			*failReason = "IK未收敛/超迭代";
		}
		return {};
	}
	(void)iters;
	return qSolved;
}

bool tcpPositionFromUrdf(
	const QString& urdfPath,
	const QString& tcpLink,
	const std::vector<double>& q,
	double outPos[3],
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
	if (!linkWorldByName.contains(tcpLink))
	{
		return false;
	}
	const osg::Vec3d t = linkWorldByName.value(tcpLink).getTrans();
	outPos[0] = t.x();
	outPos[1] = t.y();
	outPos[2] = t.z();
	if (outRot)
	{
		*outRot = linkWorldByName.value(tcpLink).getRotate();
		normalizeQuatSafe(*outRot);
	}
	return true;
}

bool currentTcpPositionFromInstruction(const RobotInstruction::Base& cmd, double outPos[3])
{
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	const auto itTcp = ext.find("context.tcpLinkName");
	if (itUrdf == ext.end() || itTcp == ext.end() || itUrdf->second.empty() || itTcp->second.empty())
	{
		return false;
	}
	const std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		return false;
	}
	return tcpPositionFromUrdf(QString::fromStdString(itUrdf->second), QString::fromStdString(itTcp->second), q, outPos);
}

void logIkSolveResidual(
	const RobotInstruction::Base& cmd,
	const std::vector<double>& qSolved,
	const char* plannerName)
{
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	const auto itTcp = ext.find("context.tcpLinkName");
	if (itUrdf == ext.end() || itTcp == ext.end() || itUrdf->second.empty() || itTcp->second.empty())
	{
		return;
	}
	if (qSolved.empty() || !cmd.hasPoseProperty())
	{
		return;
	}
	const QString urdfPath = QString::fromStdString(itUrdf->second);
	const QString tcpLink = QString::fromStdString(itTcp->second);
	const RobotInstruction::Vec3 target = cmd.pose();
	const bool hasTargetEuler = cmd.hasEulerProperty();
	double fkPos[3] = { 0.0, 0.0, 0.0 };
	osg::Quat fkRot;
	if (!tcpPositionFromUrdf(urdfPath, tcpLink, qSolved, fkPos, &fkRot))
	{
		std::ostringstream os;
		os << "[IK残差] planner=" << (plannerName ? plannerName : "Unknown")
		   << ", tcpLink=" << tcpLink.toStdString()
		   << ", FK检查失败（无法基于q重算tcp位置）";
		RunLogger::warn(os.str());
		return;
	}

	std::ostringstream qPreview;
	const int previewN = std::min(6, static_cast<int>(qSolved.size()));
	for (int i = 0; i < previewN; ++i)
	{
		if (i > 0)
		{
			qPreview << ", ";
		}
		qPreview << qSolved[static_cast<size_t>(i)];
	}
	if (static_cast<int>(qSolved.size()) > previewN)
	{
		qPreview << ", ...";
	}

	const double dx = fkPos[0] - target.x;
	const double dy = fkPos[1] - target.y;
	const double dz = fkPos[2] - target.z;
	const double errMm = std::sqrt(dx * dx + dy * dy + dz * dz);
	double errRotDeg = 0.0;
	if (hasTargetEuler)
	{
		const osg::Quat targetQuat = eulerDegToQuat(cmd.eulerDeg());
		double eRot[3] = { 0.0, 0.0, 0.0 };
		quatErrorAxisAngle(fkRot, targetQuat, eRot);
		const double errRotRad = std::sqrt(eRot[0] * eRot[0] + eRot[1] * eRot[1] + eRot[2] * eRot[2]);
		errRotDeg = errRotRad * (180.0 / 3.14159265358979323846);
	}

	std::ostringstream os;
	os << "[IK残差] planner=" << (plannerName ? plannerName : "Unknown")
	   << ", tcpLink=" << tcpLink.toStdString()
	   << ", targetPose=(" << target.x << ", " << target.y << ", " << target.z << ")"
	   << ", solvedQ[0..]=[" << qPreview.str() << "]"
	   << ", fkTcp=(" << fkPos[0] << ", " << fkPos[1] << ", " << fkPos[2] << ")"
	   << ", errMm=" << errMm;
	if (hasTargetEuler)
	{
		os << ", errRotDeg=" << errRotDeg;
	}
	RunLogger::info(os.str());
}

std::vector<double> solveTargetByUrdfNumericalIkIfPossible(const RobotInstruction::Base& cmd, std::string* failReason)
{
	if (!cmd.hasPoseProperty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const auto& ext = cmd.extensionProperties();
	const auto itUrdf = ext.find("context.urdfPath");
	if (itUrdf == ext.end() || itUrdf->second.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const auto itTcp = ext.find("context.tcpLinkName");
	if (itTcp == ext.end() || itTcp->second.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	std::vector<double> q = currentJointVectorFromInstruction(cmd);
	if (q.empty())
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const QString urdfPath = QString::fromStdString(itUrdf->second);
	const QString tcpLink = QString::fromStdString(itTcp->second);
	const RobotInstruction::Vec3 p = cmd.pose();
	const double target[3] = { p.x, p.y, p.z };
	const bool useOrientation = cmd.hasEulerProperty();
	const osg::Quat targetQuat = useOrientation ? eulerDegToQuat(cmd.eulerDeg()) : osg::Quat();
	const double targetNormMm = std::sqrt(target[0] * target[0] + target[1] * target[1] + target[2] * target[2]);
	if (targetNormMm > 50000.0)
	{
		if (failReason)
		{
			*failReason = "目标越界/单位不一致";
		}
		return {};
	}

	double pos[3] = { 0.0, 0.0, 0.0 };
	osg::Quat curQuat;
	if (!tcpPositionFromUrdf(urdfPath, tcpLink, q, pos, useOrientation ? &curQuat : nullptr))
	{
		if (failReason)
		{
			*failReason = "无DH上下文";
		}
		return {};
	}
	const double initialErr = std::sqrt(
		(target[0] - pos[0]) * (target[0] - pos[0]) +
		(target[1] - pos[1]) * (target[1] - pos[1]) +
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
	const int maxIters = 180;
	const int taskDim = useOrientation ? 6 : 3;
	// Unit balancing for 6D IK: translation is in mm, rotation is in rad.
	// Without scaling, orientation residual is numerically too small.
	const double orientationWeight = useOrientation ? 300.0 : 1.0;
	for (int iter = 0; iter < maxIters; ++iter)
	{
		if (!tcpPositionFromUrdf(urdfPath, tcpLink, q, pos, useOrientation ? &curQuat : nullptr))
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
		double posErr = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
		double rotErr = 0.0;
		if (useOrientation)
		{
			double eRot[3] = { 0.0, 0.0, 0.0 };
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
			double p2[3] = { 0.0, 0.0, 0.0 };
			osg::Quat q2;
			if (!tcpPositionFromUrdf(urdfPath, tcpLink, qPert, p2, useOrientation ? &q2 : nullptr))
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
				double dRot[3] = { 0.0, 0.0, 0.0 };
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

	if (failReason)
	{
		*failReason = "IK未收敛/超迭代";
	}
	return {};
}

class PtpPlanner final : public RobotInstruction::PlannerBase
{
public:
	explicit PtpPlanner(const std::vector<robot_kinematics::DhRow>* dhRows)
		: m_dhRows(dhRows)
	{
	}

	bool canHandle(RobotInstruction::Type type) const override { return type == RobotInstruction::Type::PTP; }

	bool validate(const RobotInstruction::Base& cmd, std::string* errMsg) const override
	{
		if (!cmd.hasSpeedProperty() || cmd.speed() <= 0.0)
		{
			if (errMsg)
				*errMsg = "PTP instruction requires speed > 0.";
			return false;
		}
		if (!cmd.hasAccelProperty() || cmd.accel() <= 0.0)
		{
			if (errMsg)
				*errMsg = "PTP instruction requires acceleration > 0.";
			return false;
		}
		return true;
	}

	bool plan(const RobotInstruction::Base& cmd, RobotInstruction::PlanResult& out, std::string* errMsg) const override
	{
		if (!validate(cmd, errMsg))
		{
			return false;
		}
		const std::vector<double> q0 = currentJointVectorFromInstruction(cmd);
		if (q0.empty())
		{
			if (errMsg)
			{
				*errMsg = "无DH上下文";
			}
			return false;
		}
		std::vector<double> targetQ;
		std::string ikFailReason;
		const bool preferUrdfIk = hasTcpLinkContext(cmd);
		if (preferUrdfIk)
		{
			targetQ = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
		}
		if (targetQ.empty() && m_dhRows && !m_dhRows->empty())
		{
			targetQ = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
		}
		if (targetQ.empty() && !preferUrdfIk)
		{
			targetQ = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
		}
		if (targetQ.empty())
		{
			targetQ = solveTargetByLegacyJointDelta(cmd);
		}
		if (targetQ.empty())
		{
			if (errMsg)
			{
				*errMsg = ikFailReason.empty() ? "无DH上下文" : ikFailReason;
			}
			return false;
		}
		if (targetQ.size() != q0.size())
		{
			if (errMsg)
			{
				*errMsg = "IK未收敛/超迭代";
			}
			return false;
		}
		double maxJointDelta = 0.0;
		for (size_t i = 0; i < targetQ.size(); ++i)
		{
			maxJointDelta = std::max(maxJointDelta, std::abs(targetQ[i] - q0[i]));
		}
		const double vmax = std::max(1e-6, cmd.speed() * kDegToRad);
		const double amax = std::max(1e-6, cmd.accel() * kDegToRad);
		const double durationSec = trapezoidDuration(maxJointDelta, vmax, amax);

		out.ok = true;
		out.plannerName = "PtpPlanner";
		out.summary = "PTP solved with joint target output.";
		out.durationSec = durationSec;
		out.jointTargetsRad = targetQ;
		out.jointTrajectoryRad = { targetQ };
		logIkSolveResidual(cmd, targetQ, "PtpPlanner");
		return true;
	}

private:
	const std::vector<robot_kinematics::DhRow>* m_dhRows = nullptr;
};

class LinePlanner final : public RobotInstruction::PlannerBase
{
public:
	explicit LinePlanner(const std::vector<robot_kinematics::DhRow>* dhRows)
		: m_dhRows(dhRows)
	{
	}

	bool canHandle(RobotInstruction::Type type) const override { return type == RobotInstruction::Type::LINE; }

	bool validate(const RobotInstruction::Base& cmd, std::string* errMsg) const override
	{
		if (!cmd.hasPoseProperty())
		{
			if (errMsg)
				*errMsg = "LINE instruction requires target pose.";
			return false;
		}
		if (!cmd.hasSpeedProperty() || cmd.speed() <= 0.0)
		{
			if (errMsg)
				*errMsg = "LINE instruction requires speed > 0.";
			return false;
		}
		if (!cmd.hasAccelProperty() || cmd.accel() <= 0.0)
		{
			if (errMsg)
				*errMsg = "LINE instruction requires acceleration > 0.";
			return false;
		}
		if (cmd.hasBlendRadiusProperty() && cmd.blendRadius() < 0.0)
		{
			if (errMsg)
				*errMsg = "LINE instruction blend radius must be >= 0.";
			return false;
		}
		return true;
	}

	bool plan(const RobotInstruction::Base& cmd, RobotInstruction::PlanResult& out, std::string* errMsg) const override
	{
		if (!validate(cmd, errMsg))
		{
			return false;
		}

		std::vector<double> q0 = currentJointVectorFromInstruction(cmd);
		if (q0.empty())
		{
			if (errMsg)
				*errMsg = "LINE planning failed: missing context.currentJointRadCsv.";
			return false;
		}

		std::vector<double> qTarget;
		std::string ikFailReason;
		const bool preferUrdfIk = hasTcpLinkContext(cmd);
		if (preferUrdfIk)
		{
			qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
		}
		if (qTarget.empty() && m_dhRows && !m_dhRows->empty())
		{
			qTarget = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
		}
		if (qTarget.empty() && !preferUrdfIk)
		{
			qTarget = solveTargetByUrdfNumericalIkIfPossible(cmd, &ikFailReason);
		}
		if (qTarget.empty())
		{
			qTarget = solveTargetByLegacyJointDelta(cmd);
		}
		if (qTarget.empty() || qTarget.size() != q0.size())
		{
			if (errMsg)
			{
				*errMsg = ikFailReason.empty() ? "无DH上下文" : ikFailReason;
			}
			return false;
		}

		double durationSec = 0.0;
		double curTcp[3] = { 0.0, 0.0, 0.0 };
		const RobotInstruction::Vec3 p = cmd.pose();
		if (currentTcpPositionFromInstruction(cmd, curTcp))
		{
			const double dx = p.x - curTcp[0];
			const double dy = p.y - curTcp[1];
			const double dz = p.z - curTcp[2];
			const double cartDistMm = std::sqrt(dx * dx + dy * dy + dz * dz);
			durationSec = trapezoidDuration(cartDistMm, cmd.speed(), cmd.accel());
		}
		else
		{
			double maxJointDelta = 0.0;
			for (size_t i = 0; i < qTarget.size(); ++i)
			{
				maxJointDelta = std::max(maxJointDelta, std::abs(qTarget[i] - q0[i]));
			}
			durationSec = trapezoidDuration(
				maxJointDelta,
				std::max(1e-6, cmd.speed() * kDegToRad),
				std::max(1e-6, cmd.accel() * kDegToRad));
		}

		out.jointTrajectoryRad.clear();
		const int samples = 24;
		out.jointTrajectoryRad.reserve(static_cast<size_t>(samples + 1));
		for (int i = 1; i <= samples; ++i)
		{
			const double u = static_cast<double>(i) / static_cast<double>(samples);
			std::vector<double> q = q0;
			for (size_t j = 0; j < q.size(); ++j)
			{
				q[j] = q0[j] + (qTarget[j] - q0[j]) * u;
			}
			out.jointTrajectoryRad.push_back(std::move(q));
		}

		out.ok = true;
		out.plannerName = "LinePlanner";
		out.summary = "LINE solved with joint trajectory output.";
		out.durationSec = durationSec;
		out.jointTargetsRad = qTarget;
		logIkSolveResidual(cmd, qTarget, "LinePlanner");
		return true;
	}

private:
	const std::vector<robot_kinematics::DhRow>* m_dhRows = nullptr;
};
} // namespace

namespace RobotInstruction
{
void Controller::setDhRows(const std::vector<robot_kinematics::DhRow>& rows)
{
	m_dhRows = rows;
}

void Controller::clearDhRows()
{
	m_dhRows.clear();
}

bool Controller::hasDhRows() const
{
	return !m_dhRows.empty();
}

void Controller::registerPlanner(const std::shared_ptr<PlannerBase>& planner)
{
	if (planner)
	{
		m_planners.push_back(planner);
	}
}

void Controller::clearPlanners()
{
	m_planners.clear();
}

void Controller::buildDefaultPlanners()
{
	clearPlanners();
	registerPlanner(std::make_shared<PtpPlanner>(&m_dhRows));
	registerPlanner(std::make_shared<LinePlanner>(&m_dhRows));
}

bool Controller::validate(const Base& cmd, std::string* errMsg) const
{
	const PlannerBase* planner = findPlanner(cmd.type());
	if (!planner)
	{
		if (errMsg)
		{
			*errMsg = "No planner is registered for instruction type.";
		}
		return false;
	}
	return planner->validate(cmd, errMsg);
}

bool Controller::plan(const Base& cmd, PlanResult& out, std::string* errMsg) const
{
	const PlannerBase* planner = findPlanner(cmd.type());
	if (!planner)
	{
		if (errMsg)
		{
			*errMsg = "No planner is registered for instruction type.";
		}
		return false;
	}
	return planner->plan(cmd, out, errMsg);
}

const PlannerBase* Controller::findPlanner(Type t) const
{
	const auto it = std::find_if(m_planners.begin(), m_planners.end(), [t](const std::shared_ptr<PlannerBase>& p) {
		return p && p->canHandle(t);
	});
	if (it == m_planners.end())
	{
		return nullptr;
	}
	return it->get();
}
} // namespace RobotInstruction
