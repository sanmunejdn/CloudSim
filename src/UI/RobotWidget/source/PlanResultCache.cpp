#include "PlanResultCache.h"

QString PlanResultCache::makeKey(const QString& id, const QString& fp)
{
	return id + QLatin1Char('|') + fp;
}

const RobotInstruction::PlanResult* PlanResultCache::fetch(
	const QString& instructionId,
	const QString& fingerprint) const
{
	const auto it = m_entries.find(makeKey(instructionId, fingerprint));
	return (it != m_entries.end()) ? &it->result : nullptr;
}

void PlanResultCache::store(
	const QString& instructionId,
	const QString& fingerprint,
	const RobotInstruction::PlanResult& result)
{
	Entry e;
	e.instructionId = instructionId;
	e.fingerprint = fingerprint;
	e.result = result;
	m_entries.insert(makeKey(instructionId, fingerprint), std::move(e));
}

void PlanResultCache::invalidateByInstruction(const QString& instructionId)
{
	auto it = m_entries.begin();
	while (it != m_entries.end())
	{
		if (it->instructionId == instructionId)
		{
			it = m_entries.erase(it);
		}
		else
		{
			++it;
		}
	}
}

void PlanResultCache::invalidateAll()
{
	m_entries.clear();
}
