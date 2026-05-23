#include "RobotProgramStore.h"

void RobotProgramStore::clear()
{
	m_programs.clear();
	m_labels.clear();
	m_backendIds.clear();
	m_activeInstanceIndex = 0;
}

QStringList RobotProgramStore::robotBackendIds() const
{
	return m_backendIds;
}

QStringList RobotProgramStore::robotLabels() const
{
	return m_labels;
}

QString RobotProgramStore::activeRobotBackendId() const
{
	if (m_activeInstanceIndex >= 0 && m_activeInstanceIndex < m_backendIds.size())
	{
		return m_backendIds[m_activeInstanceIndex];
	}
	return QString();
}

void RobotProgramStore::setActiveInstanceIndex(const int index)
{
	if (index < 0 || index >= m_backendIds.size())
	{
		return;
	}
	m_activeInstanceIndex = index;
	ensureActiveProgram();
}

void RobotProgramStore::setActiveRobotBackendId(const QString& sceneBackendId)
{
	const int idx = m_backendIds.indexOf(sceneBackendId);
	if (idx >= 0)
	{
		setActiveInstanceIndex(idx);
	}
}

void RobotProgramStore::setRobotInstances(const QStringList& labels, const QStringList& backendIds)
{
	m_labels = labels;
	m_backendIds = backendIds;
	if (m_activeInstanceIndex >= m_backendIds.size())
	{
		m_activeInstanceIndex = m_backendIds.isEmpty() ? 0 : m_backendIds.size() - 1;
	}
	for (const QString& id : m_backendIds)
	{
		if (!m_programs.contains(id))
		{
			m_programs.insert(id, {});
		}
	}
	ensureActiveProgram();
}

std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::programFor(const QString& sceneBackendId)
{
	return m_programs[sceneBackendId];
}

const std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::programFor(
	const QString& sceneBackendId) const
{
	static const std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
	const auto it = m_programs.constFind(sceneBackendId);
	return it != m_programs.constEnd() ? it.value() : s_empty;
}

std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::activeProgram()
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		static std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
		return s_empty;
	}
	return programFor(id);
}

const std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::activeProgram() const
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		static const std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
		return s_empty;
	}
	return programFor(id);
}

void RobotProgramStore::setProgramFor(
	const QString& sceneBackendId,
	std::vector<std::shared_ptr<RobotInstruction::Base>> program)
{
	m_programs.insert(sceneBackendId, std::move(program));
}

void RobotProgramStore::ensureActiveProgram()
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		return;
	}
	if (!m_programs.contains(id))
	{
		m_programs.insert(id, {});
	}
}
