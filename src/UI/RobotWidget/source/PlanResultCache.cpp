/// @file PlanResultCache.cpp
/// @brief PlanResultCache 实现

#include "PlanResultCache.h"

QString PlanResultCache::makeKey(const QString& id, const QString& fp)
{
	return id + QLatin1Char('|') + fp;
}

const RobotInstruction::PlanResult* PlanResultCache::fetch(const QString& instructionId,
														   const QString& fingerprint) const
{
	const auto it = m_entries.find(makeKey(instructionId, fingerprint));
	return (it != m_entries.end()) ? &it->result : nullptr;
}

void PlanResultCache::evictOverflow()
{
	while (m_entries.size() > m_maxEntries && !m_fifoKeys.isEmpty())
	{
		const QString oldest = m_fifoKeys.takeFirst();
		m_entries.remove(oldest);
	}
}

void PlanResultCache::store(const QString& instructionId, const QString& fingerprint,
							const RobotInstruction::PlanResult& result, const size_t motionIndex)
{
	const QString key = makeKey(instructionId, fingerprint);
	if (m_entries.contains(key))
	{
		m_fifoKeys.removeAll(key);
	}
	Entry e;
	e.instructionId = instructionId;
	e.fingerprint = fingerprint;
	e.motionIndex = motionIndex;
	e.result = result;
	// 播放只留目标关节，避免万级点堆积笛卡尔采样轨迹
	e.result.jointTrajectoryRad.clear();
	m_entries.insert(key, std::move(e));
	m_fifoKeys.append(key);
	evictOverflow();
}

void PlanResultCache::invalidateByInstruction(const QString& instructionId)
{
	auto it = m_entries.begin();
	while (it != m_entries.end())
	{
		if (it->instructionId == instructionId)
		{
			m_fifoKeys.removeAll(it.key());
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
	m_fifoKeys.clear();
}

void PlanResultCache::evictFarBehind(const size_t currentMotionIndex, const size_t keepBehind)
{
	if (currentMotionIndex <= keepBehind)
	{
		return;
	}
	const size_t cutoff = currentMotionIndex - keepBehind;
	auto it = m_entries.begin();
	while (it != m_entries.end())
	{
		if (it->motionIndex < cutoff)
		{
			m_fifoKeys.removeAll(it.key());
			it = m_entries.erase(it);
		}
		else
		{
			++it;
		}
	}
}
