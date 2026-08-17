/// @file RobotProgramStore.cpp
/// @brief 程序存储

#include "RobotProgramStore.h"

void RobotProgramStore::clear()
{
	m_catalogs.clear();
	m_legacyProgramsView.clear();
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
	ensureActiveCatalog();
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
		if (!m_catalogs.contains(id))
		{
			m_catalogs.insert(id, RobotInstruction::RobotProgramCatalog::withDefaultMain());
		}
	}
	ensureActiveCatalog();
}

RobotInstruction::RobotProgramCatalog& RobotProgramStore::catalogFor(const QString& sceneBackendId)
{
	if (!m_catalogs.contains(sceneBackendId))
	{
		m_catalogs.insert(sceneBackendId, RobotInstruction::RobotProgramCatalog::withDefaultMain());
	}
	return m_catalogs[sceneBackendId];
}

const RobotInstruction::RobotProgramCatalog& RobotProgramStore::catalogFor(const QString& sceneBackendId) const
{
	static const RobotInstruction::RobotProgramCatalog s_empty =
		RobotInstruction::RobotProgramCatalog::withDefaultMain();
	const auto it = m_catalogs.constFind(sceneBackendId);
	return it != m_catalogs.constEnd() ? it.value() : s_empty;
}

RobotInstruction::RobotProgramCatalog& RobotProgramStore::activeCatalog()
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		static RobotInstruction::RobotProgramCatalog s_empty = RobotInstruction::RobotProgramCatalog::withDefaultMain();
		return s_empty;
	}
	return catalogFor(id);
}

const RobotInstruction::RobotProgramCatalog& RobotProgramStore::activeCatalog() const
{
	return const_cast<RobotProgramStore*>(this)->activeCatalog();
}

std::string RobotProgramStore::activeProgramIdUtf8() const
{
	return activeCatalog().activeProgramId();
}

void RobotProgramStore::setActiveProgramIdUtf8(const std::string& programId)
{
	activeCatalog().setActiveProgramId(programId);
}

std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::programFor(const QString& sceneBackendId)
{
	return catalogFor(sceneBackendId).activeSteps();
}

const std::vector<std::shared_ptr<RobotInstruction::Base>>&
RobotProgramStore::programFor(const QString& sceneBackendId) const
{
	return catalogFor(sceneBackendId).activeSteps();
}

std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::activeProgram()
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		static std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
		return s_empty;
	}
	return catalogFor(id).activeSteps();
}

const std::vector<std::shared_ptr<RobotInstruction::Base>>& RobotProgramStore::activeProgram() const
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		static const std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
		return s_empty;
	}
	return catalogFor(id).activeSteps();
}

void RobotProgramStore::setProgramFor(const QString& sceneBackendId,
									  std::vector<std::shared_ptr<RobotInstruction::Base>> program)
{
	auto& catalog = catalogFor(sceneBackendId);
	if (RobotInstruction::RobotProgram* mainProg = catalog.mainProgram())
	{
		mainProg->steps = std::move(program);
	}
}

const QHash<QString, std::vector<std::shared_ptr<RobotInstruction::Base>>>& RobotProgramStore::allPrograms() const
{
	m_legacyProgramsView.clear();
	for (auto it = m_catalogs.constBegin(); it != m_catalogs.constEnd(); ++it)
	{
		m_legacyProgramsView.insert(it.key(), it.value().activeSteps());
	}
	return m_legacyProgramsView;
}

void RobotProgramStore::ensureActiveCatalog()
{
	const QString id = activeRobotBackendId();
	if (id.isEmpty())
	{
		return;
	}
	if (!m_catalogs.contains(id))
	{
		m_catalogs.insert(id, RobotInstruction::RobotProgramCatalog::withDefaultMain());
	}
}
