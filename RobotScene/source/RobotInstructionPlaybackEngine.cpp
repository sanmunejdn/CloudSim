#include "RobotInstructionPlaybackEngine.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"
#include "RobotSceneKinematics.h"

#include "RunLogger.h"
#include "UrdfRobotLoader.h"

#include <QByteArray>
#include <QHash>
#include <QString>

#include <algorithm>
#include <cmath>

using namespace RobotSimulation;

namespace
{
std::string qToUtf8Std(const QString& s)
{
	const QByteArray utf8 = s.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}
} // namespace

void RobotInstructionPlaybackEngine::stop()
{
	m_running = false;
	m_queue.clear();
	m_planResults.clear();
	m_segmentIndex = 0;
	m_jointAnglesRad.clear();
	m_segStartJointAngles.clear();
	m_fkMeshWorldT0.clear();
	m_outerWorldAtStart.clear();
	m_segAngleStartRad = 0.0;
	m_segAngleTargetRad = 0.0;
	m_segDurationSec = 0.0;
}

bool RobotInstructionPlaybackEngine::tryStart(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	const QVector<RobotSimulationCommand>& queue,
	const QVector<double>& initialJointAnglesRad,
	QString* errorOut)
{
	stop();
	if (!doc || !osg)
	{
		RunLogger::warn("RobotInstructionPlaybackEngine::tryStart failed: null document or viewer.");
		if (errorOut)
		{
			*errorOut = QStringLiteral("No document or viewer.");
		}
		return false;
	}
	if (!doc->hasRobotSimulationContext())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine::tryStart failed: no robot simulation context.");
		if (errorOut)
		{
			*errorOut = QStringLiteral("No robot context.");
		}
		return false;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePath();
	if (urdfPath.isEmpty())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine::tryStart failed: empty URDF path.");
		if (errorOut)
		{
			*errorOut = QStringLiteral("Empty URDF path.");
		}
		return false;
	}
	const QStringList jnames = doc->robotRevoluteJointNames();
	if (jnames.isEmpty())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine::tryStart failed: no revolute joints.");
		if (errorOut)
		{
			*errorOut = QStringLiteral("No revolute joints.");
		}
		return false;
	}
	if (queue.isEmpty())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine::tryStart failed: empty command queue.");
		if (errorOut)
		{
			*errorOut = QStringLiteral("Empty command queue.");
		}
		return false;
	}

	m_jointAnglesRad.resize(jnames.size());
	if (initialJointAnglesRad.size() == jnames.size())
	{
		m_jointAnglesRad = initialJointAnglesRad;
	}
	else
	{
		m_jointAnglesRad.fill(0.0);
	}

	QString fkErr;
	if (doc->hasRobotKinematicsBind())
	{
		m_fkMeshWorldT0 = doc->robotFkMeshWorldT0();
		m_outerWorldAtStart.clear();
		for (auto it = doc->robotOuterWorldAtBind().constBegin(); it != doc->robotOuterWorldAtBind().constEnd(); ++it)
		{
			m_outerWorldAtStart[it.key().toStdString()] = it.value();
		}
	}
	else
	{
		const QVector<double> zeros = QVector<double>(jnames.size(), 0.0);
		if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, zeros, m_fkMeshWorldT0, &fkErr))
		{
			RunLogger::warn(qToUtf8Std(QStringLiteral("RobotInstructionPlaybackEngine::tryStart FK init failed: %1")
				.arg(fkErr.isEmpty() ? QStringLiteral("Forward kinematics failed.") : fkErr)));
			if (errorOut)
			{
				*errorOut = fkErr.isEmpty() ? QStringLiteral("Forward kinematics failed.") : fkErr;
			}
			stop();
			return false;
		}
		m_outerWorldAtStart.clear();
		for (auto it = doc->robotLinkNameToBackendId().constBegin(); it != doc->robotLinkNameToBackendId().constEnd(); ++it)
		{
			osg::Matrixd M;
			if (osg->getBackendRootWorldMatrix(it.value().toStdString(), M))
			{
				m_outerWorldAtStart[it.value().toStdString()] = M;
			}
		}
	}

	m_queue = queue;
	m_segmentIndex = 0;
	const RobotSimulationCommand& c0 = m_queue[0];
	if (c0.jointIndex < 0 || c0.jointIndex >= m_jointAnglesRad.size())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine::tryStart failed: invalid joint index in first command.");
		if (errorOut)
		{
			*errorOut = QStringLiteral("Invalid joint index in first command.");
		}
		stop();
		return false;
	}

	m_segAngleStartRad = m_jointAnglesRad[c0.jointIndex];
	m_segAngleTargetRad = m_segAngleStartRad + c0.angleDeg * (kPi / 180.0);
	m_segAngleTargetRad = resolveSegmentTargetRad(0, m_segAngleTargetRad);
	m_segDurationSec = std::max(0.05, c0.durationSec);
	m_segStartJointAngles = m_jointAnglesRad;
	m_segmentTimer.restart();
	m_running = true;
	RunLogger::info("RobotInstructionPlaybackEngine started.");
	return true;
}

bool RobotInstructionPlaybackEngine::tryStartFromPlanResults(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	const QVector<RobotSimulationCommand>& legacyQueue,
	const std::vector<RobotInstruction::PlanResult>& planResults,
	const QVector<double>& initialJointAnglesRad,
	QString* errorOut)
{
	if (!tryStart(doc, osg, legacyQueue, initialJointAnglesRad, errorOut))
	{
		return false;
	}
	m_planResults = planResults;
	m_segStartJointAngles = m_jointAnglesRad;
	m_segAngleTargetRad = resolveSegmentTargetRad(0, m_segAngleTargetRad);
	if (!m_queue.isEmpty())
	{
		m_segDurationSec = std::max(0.05, m_queue[0].durationSec);
		if (!m_planResults.empty() && m_planResults.front().durationSec > 1e-6)
		{
			m_segDurationSec = m_planResults.front().durationSec;
		}
	}
	RunLogger::info("RobotInstructionPlaybackEngine started with controller plan results adapter.");
	return true;
}

RobotInstructionPlaybackTickResult RobotInstructionPlaybackEngine::tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg)
{
	if (!m_running)
	{
		return RobotInstructionPlaybackTickResult::Continue;
	}
	if (!osg || !doc || !doc->hasRobotSimulationContext())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine aborted: invalid runtime context.");
		stop();
		return RobotInstructionPlaybackTickResult::Aborted;
	}
	if (doc->robotRevoluteJointNames().isEmpty())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine aborted: robot has no revolute joints.");
		stop();
		return RobotInstructionPlaybackTickResult::Aborted;
	}
	if (m_segmentIndex < 0 || m_segmentIndex >= m_queue.size())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine aborted: segment index out of range.");
		stop();
		return RobotInstructionPlaybackTickResult::Aborted;
	}

	const RobotSimulationCommand& cmd = m_queue[m_segmentIndex];
	const int jidx = cmd.jointIndex;
	if (jidx < 0 || jidx >= m_jointAnglesRad.size())
	{
		RunLogger::warn("RobotInstructionPlaybackEngine aborted: command joint index out of range.");
		stop();
		return RobotInstructionPlaybackTickResult::Aborted;
	}

	const double elapsed = m_segmentTimer.elapsed() / 1000.0;
	const double segDuration = (m_segDurationSec > 1e-9) ? m_segDurationSec : std::max(0.05, cmd.durationSec);
	const double u = std::min(1.0, elapsed / segDuration);
	if (!applyPlannedJointStateAtProgress(u))
	{
		const double angleRad = m_segAngleStartRad + u * (m_segAngleTargetRad - m_segAngleStartRad);
		m_jointAnglesRad[jidx] = angleRad;
	}

	// 动态层级 / 多机器人：无 link→backend 烘焙映射时，直接按关节角更新 MatrixTransform
	if (doc->robotLinkNameToBackendId().isEmpty())
	{
		if (!RobotSceneKinematics::applyJointAnglesFromDocument(doc, osg, m_jointAnglesRad))
		{
			RunLogger::warn("RobotInstructionPlaybackEngine aborted: applyJointAnglesFromDocument failed.");
			stop();
			return RobotInstructionPlaybackTickResult::Aborted;
		}
	}
	else
	{
		const QString urdfPath = doc->robotUrdfAbsolutePath();
		QHash<QString, osg::Matrixd> Tq;
		QString fkErr;
		if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, m_jointAnglesRad, Tq, &fkErr))
		{
			RunLogger::warn(qToUtf8Std(QStringLiteral("RobotInstructionPlaybackEngine aborted: computeMeshWorldMatrices failed: %1")
				.arg(fkErr)));
			stop();
			return RobotInstructionPlaybackTickResult::Aborted;
		}

		RobotSceneKinematics::applyMeshWorldMatricesRelativeToBind(
			osg, Tq, m_fkMeshWorldT0, doc->robotLinkNameToBackendId(), m_outerWorldAtStart);
	}

	if (u >= 1.0 - 1e-9)
	{
		if (!applyPlannedJointStateAtProgress(1.0))
		{
			m_jointAnglesRad[jidx] = m_segAngleTargetRad;
		}
		m_segmentIndex++;
		if (m_segmentIndex >= m_queue.size())
		{
			m_running = false;
			RunLogger::info("RobotInstructionPlaybackEngine finished all commands.");
			return RobotInstructionPlaybackTickResult::Finished;
		}
		const RobotSimulationCommand& next = m_queue[m_segmentIndex];
		const int nj = next.jointIndex;
		if (nj < 0 || nj >= m_jointAnglesRad.size())
		{
			RunLogger::warn("RobotInstructionPlaybackEngine aborted: next command joint index out of range.");
			stop();
			return RobotInstructionPlaybackTickResult::Aborted;
		}
		m_segAngleStartRad = m_jointAnglesRad[nj];
		m_segAngleTargetRad = m_segAngleStartRad + next.angleDeg * (kPi / 180.0);
		m_segAngleTargetRad = resolveSegmentTargetRad(m_segmentIndex, m_segAngleTargetRad);
		m_segDurationSec = std::max(0.05, next.durationSec);
		const RobotInstruction::PlanResult* plan = currentPlanResult();
		if (plan && plan->durationSec > 1e-6)
		{
			m_segDurationSec = plan->durationSec;
		}
		startNewSegmentFromCurrentState();
	}

	return RobotInstructionPlaybackTickResult::Continue;
}

double RobotInstructionPlaybackEngine::resolveSegmentTargetRad(int segmentIndex, double defaultTargetRad) const
{
	if (segmentIndex < 0 || segmentIndex >= static_cast<int>(m_planResults.size()))
	{
		return defaultTargetRad;
	}
	const RobotInstruction::PlanResult& plan = m_planResults[segmentIndex];
	if (!plan.ok || plan.jointTargetsRad.empty())
	{
		return defaultTargetRad;
	}
	return plan.jointTargetsRad.front();
}

const RobotInstruction::PlanResult* RobotInstructionPlaybackEngine::currentPlanResult() const
{
	if (m_segmentIndex < 0 || m_segmentIndex >= static_cast<int>(m_planResults.size()))
	{
		return nullptr;
	}
	return &m_planResults[static_cast<size_t>(m_segmentIndex)];
}

bool RobotInstructionPlaybackEngine::applyPlannedJointStateAtProgress(double u)
{
	const RobotInstruction::PlanResult* plan = currentPlanResult();
	if (!plan || !plan->ok)
	{
		return false;
	}
	const double uc = std::clamp(u, 0.0, 1.0);
	if (!plan->jointTrajectoryRad.empty())
	{
		const auto& traj = plan->jointTrajectoryRad;
		if (traj.empty() || traj.front().size() != static_cast<size_t>(m_jointAnglesRad.size()))
		{
			return false;
		}
		// Treat planned trajectory as waypoints after the current segment start.
		// This guarantees visible interpolation even when planner only outputs one target waypoint.
		if (m_segStartJointAngles.size() != m_jointAnglesRad.size())
		{
			// Fallback to legacy interpretation when start snapshot is unavailable.
			if (traj.size() == 1U)
			{
				for (int j = 0; j < m_jointAnglesRad.size(); ++j)
				{
					m_jointAnglesRad[j] = traj[0][static_cast<size_t>(j)];
				}
				return true;
			}
			const double scaled = uc * static_cast<double>(traj.size() - 1U);
			const size_t i0 = static_cast<size_t>(std::floor(scaled));
			const size_t i1 = std::min(i0 + 1U, traj.size() - 1U);
			const double t = scaled - static_cast<double>(i0);
			if (traj[i0].size() != static_cast<size_t>(m_jointAnglesRad.size())
				|| traj[i1].size() != static_cast<size_t>(m_jointAnglesRad.size()))
			{
				return false;
			}
			for (int j = 0; j < m_jointAnglesRad.size(); ++j)
			{
				const double q0 = traj[i0][static_cast<size_t>(j)];
				const double q1 = traj[i1][static_cast<size_t>(j)];
				m_jointAnglesRad[j] = q0 + (q1 - q0) * t;
			}
			return true;
		}

		const size_t waypoints = traj.size() + 1U; // [start] + planner waypoints
		const double scaled = uc * static_cast<double>(waypoints - 1U);
		const size_t i0 = static_cast<size_t>(std::floor(scaled));
		const size_t i1 = std::min(i0 + 1U, waypoints - 1U);
		const double t = scaled - static_cast<double>(i0);

		const auto readQ = [&](size_t idx, int joint) {
			if (idx == 0U)
			{
				return m_segStartJointAngles[joint];
			}
			return traj[idx - 1U][static_cast<size_t>(joint)];
		};
		if (i0 > 0U && traj[i0 - 1U].size() != static_cast<size_t>(m_jointAnglesRad.size()))
		{
			return false;
		}
		if (i1 > 0U && traj[i1 - 1U].size() != static_cast<size_t>(m_jointAnglesRad.size()))
		{
			return false;
		}
		for (int j = 0; j < m_jointAnglesRad.size(); ++j)
		{
			const double q0 = readQ(i0, j);
			const double q1 = readQ(i1, j);
			m_jointAnglesRad[j] = q0 + (q1 - q0) * t;
		}
		return true;
	}

	if (!plan->jointTargetsRad.empty() && plan->jointTargetsRad.size() == static_cast<size_t>(m_jointAnglesRad.size())
		&& m_segStartJointAngles.size() == m_jointAnglesRad.size())
	{
		for (int j = 0; j < m_jointAnglesRad.size(); ++j)
		{
			const double q0 = m_segStartJointAngles[j];
			const double q1 = plan->jointTargetsRad[static_cast<size_t>(j)];
			m_jointAnglesRad[j] = q0 + (q1 - q0) * uc;
		}
		return true;
	}
	return false;
}

void RobotInstructionPlaybackEngine::startNewSegmentFromCurrentState()
{
	m_segStartJointAngles = m_jointAnglesRad;
	m_segmentTimer.restart();
}
