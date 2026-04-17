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
	m_segmentIndex = 0;
	m_jointAnglesRad.clear();
	m_fkMeshWorldT0.clear();
	m_outerWorldAtStart.clear();
	m_segAngleStartRad = 0.0;
	m_segAngleTargetRad = 0.0;
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
	m_segmentTimer.restart();
	m_running = true;
	RunLogger::info("RobotInstructionPlaybackEngine started.");
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
	const double u = cmd.durationSec > 1e-9 ? std::min(1.0, elapsed / cmd.durationSec) : 1.0;
	const double angleRad = m_segAngleStartRad + u * (m_segAngleTargetRad - m_segAngleStartRad);
	m_jointAnglesRad[jidx] = angleRad;

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
		m_jointAnglesRad[jidx] = m_segAngleTargetRad;
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
		m_segmentTimer.restart();
	}

	return RobotInstructionPlaybackTickResult::Continue;
}
