/// @file ControlContext.cpp
/// @brief 外置控制器关节缓冲与场景应用

#include "ControlContext.h"

#include "RobotSceneKinematics.h"

void ControlContext::reset()
{
	QMutexLocker lock(&m_mutex);
	m_robotInstanceIndex = 0;
	m_jointNames.clear();
	m_pendingTargets.clear();
	m_hasPending = false;
	m_sensorSnapshot.clear();
}

void ControlContext::setRobotInstanceIndex(int instanceIndex)
{
	QMutexLocker lock(&m_mutex);
	m_robotInstanceIndex = instanceIndex;
}

int ControlContext::robotInstanceIndex() const
{
	QMutexLocker lock(&m_mutex);
	return m_robotInstanceIndex;
}

void ControlContext::setJointNames(const QStringList& names)
{
	QMutexLocker lock(&m_mutex);
	m_jointNames = names;
}

QStringList ControlContext::jointNames() const
{
	QMutexLocker lock(&m_mutex);
	return m_jointNames;
}

int ControlContext::jointCount() const
{
	QMutexLocker lock(&m_mutex);
	return m_jointNames.size();
}

void ControlContext::setPendingTargets(const QVector<double>& targetJointRad)
{
	QMutexLocker lock(&m_mutex);
	m_pendingTargets = targetJointRad;
	m_hasPending = true;
}

bool ControlContext::hasPendingTargets() const
{
	QMutexLocker lock(&m_mutex);
	return m_hasPending;
}

QVector<double> ControlContext::pendingTargets() const
{
	QMutexLocker lock(&m_mutex);
	return m_pendingTargets;
}

void ControlContext::clearPendingTargets()
{
	QMutexLocker lock(&m_mutex);
	m_hasPending = false;
	m_pendingTargets.clear();
}

void ControlContext::setSensorSnapshot(const QVector<double>& actualJointRad)
{
	QMutexLocker lock(&m_mutex);
	m_sensorSnapshot = actualJointRad;
}

QVector<double> ControlContext::sensorSnapshot() const
{
	QMutexLocker lock(&m_mutex);
	return m_sensorSnapshot;
}

bool ControlContext::sampleJointMetaFromDocument(IRobotSimulationDocument* doc)
{
	if (!doc)
		return false;
	const int idx = robotInstanceIndex();
	const int n = doc->robotRevoluteJointCountForInstance(idx);
	if (n <= 0)
		return false;

	QStringList names;
	if (idx == 0)
	{
		names = doc->robotRevoluteJointNames();
	}
	else
	{
		names.reserve(n);
		for (int i = 0; i < n; ++i)
			names.append(QStringLiteral("j%1").arg(i));
	}
	if (names.size() != n)
	{
		names.clear();
		names.reserve(n);
		for (int i = 0; i < n; ++i)
			names.append(QStringLiteral("j%1").arg(i));
	}

	QMutexLocker lock(&m_mutex);
	m_jointNames = names;
	if (m_sensorSnapshot.size() != n)
		m_sensorSnapshot = QVector<double>(n, 0.0);
	return true;
}

bool ControlContext::applyPendingToScene(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
										 QVector<double>& aggregatedJointAnglesRad)
{
	if (!doc || !osg)
		return false;

	QVector<double> targets;
	int instanceIndex = 0;
	{
		QMutexLocker lock(&m_mutex);
		if (!m_hasPending)
			return false;
		if (m_pendingTargets.size() != m_jointNames.size())
			return false;
		targets = m_pendingTargets;
		instanceIndex = m_robotInstanceIndex;
	}

	if (!RobotSceneKinematics::applyJointAnglesForInstance(doc, osg, instanceIndex, targets,
															aggregatedJointAnglesRad))
		return false;

	doc->noteRobotJointAnglesAppliedForInstance(instanceIndex, targets);

	// STEP_REPLY 采文档真源，禁止仅回显命令目标
	QVector<double> actual;
	if (!doc->robotLocalJointAnglesForInstance(instanceIndex, actual) || actual.size() != targets.size())
		actual = targets;

	QMutexLocker lock(&m_mutex);
	m_sensorSnapshot = actual;
	m_hasPending = false;
	m_pendingTargets.clear();
	return true;
}
