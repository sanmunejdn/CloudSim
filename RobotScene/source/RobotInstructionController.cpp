#include "RobotInstructionController.h"
#include "UrdfRobotLoader.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <QHash>
#include <QString>
#include <QStringList>

#include <osg/Matrixd>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

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

bool invert3x3(const double m[9], double invOut[9])
{
	const double c00 = m[4] * m[8] - m[5] * m[7];
	const double c01 = -(m[3] * m[8] - m[5] * m[6]);
	const double c02 = m[3] * m[7] - m[4] * m[6];
	const double c10 = -(m[1] * m[8] - m[2] * m[7]);
	const double c11 = m[0] * m[8] - m[2] * m[6];
	const double c12 = -(m[0] * m[7] - m[1] * m[6]);
	const double c20 = m[1] * m[5] - m[2] * m[4];
	const double c21 = -(m[0] * m[5] - m[2] * m[3]);
	const double c22 = m[0] * m[4] - m[1] * m[3];
	const double det = m[0] * c00 + m[1] * c01 + m[2] * c02;
	if (std::abs(det) < 1e-12)
	{
		return false;
	}
	const double invDet = 1.0 / det;
	invOut[0] = c00 * invDet;
	invOut[1] = c10 * invDet;
	invOut[2] = c20 * invDet;
	invOut[3] = c01 * invDet;
	invOut[4] = c11 * invDet;
	invOut[5] = c21 * invDet;
	invOut[6] = c02 * invDet;
	invOut[7] = c12 * invDet;
	invOut[8] = c22 * invDet;
	return true;
}

bool tcpPositionFromUrdf(
	const QString& urdfPath,
	const QString& tcpLink,
	const std::vector<double>& q,
	double outPos[3])
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
	if (!tcpPositionFromUrdf(urdfPath, tcpLink, q, pos))
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
	const int maxIters = 140;
	for (int iter = 0; iter < maxIters; ++iter)
	{
		if (!tcpPositionFromUrdf(urdfPath, tcpLink, q, pos))
		{
			if (failReason)
			{
				*failReason = "无DH上下文";
			}
			return {};
		}
		const double e[3] = { target[0] - pos[0], target[1] - pos[1], target[2] - pos[2] };
		const double errNorm = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
		if (errNorm < 1e-2)
		{
			return q;
		}

		const int n = static_cast<int>(q.size());
		std::vector<double> J(3 * n, 0.0);
		for (int j = 0; j < n; ++j)
		{
			std::vector<double> qPert = q;
			qPert[static_cast<size_t>(j)] += eps;
			double p2[3] = { 0.0, 0.0, 0.0 };
			if (!tcpPositionFromUrdf(urdfPath, tcpLink, qPert, p2))
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
		}

		double jjt[9] = { 0.0 };
		for (int r = 0; r < 3; ++r)
		{
			for (int c = 0; c < 3; ++c)
			{
				double s = 0.0;
				for (int k = 0; k < n; ++k)
				{
					s += J[r * n + k] * J[c * n + k];
				}
				jjt[r * 3 + c] = s;
			}
		}
		jjt[0] += lambda * lambda;
		jjt[4] += lambda * lambda;
		jjt[8] += lambda * lambda;
		double inv[9] = { 0.0 };
		if (!invert3x3(jjt, inv))
		{
			if (failReason)
			{
				*failReason = "IK未收敛/超迭代";
			}
			return {};
		}

		double y[3] = { 0.0, 0.0, 0.0 };
		for (int r = 0; r < 3; ++r)
		{
			y[r] = inv[r * 3 + 0] * e[0] + inv[r * 3 + 1] * e[1] + inv[r * 3 + 2] * e[2];
		}
		std::vector<double> dq(static_cast<size_t>(n), 0.0);
		for (int j = 0; j < n; ++j)
		{
			dq[static_cast<size_t>(j)] = J[0 * n + j] * y[0] + J[1 * n + j] * y[1] + J[2 * n + j] * y[2];
		}
		for (int j = 0; j < n; ++j)
		{
			dq[static_cast<size_t>(j)] = std::max(-0.2, std::min(0.2, dq[static_cast<size_t>(j)]));
			q[static_cast<size_t>(j)] += dq[static_cast<size_t>(j)];
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
		if (m_dhRows && !m_dhRows->empty())
		{
			targetQ = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
		}
		if (targetQ.empty())
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
		if (m_dhRows && !m_dhRows->empty())
		{
			qTarget = solveTargetByIkIfPossible(cmd, *m_dhRows, &ikFailReason);
		}
		if (qTarget.empty())
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
