/// @file PlanResultCache.cpp
/// @brief 规划结果与可行轴缓存

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
	while (m_feasibleAxis.size() > m_maxEntries && !m_feasibleFifoKeys.isEmpty())
	{
		const QString oldest = m_feasibleFifoKeys.takeFirst();
		m_feasibleAxis.remove(oldest);
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
	// 万级 PTP 不留轨迹；ARC/LINE 插帧依赖多样本，须保留
	if (result.plannerName != "ArcPlanner" && result.plannerName != "LinePlanner")
	{
		e.result.jointTrajectoryRad.clear();
	}
	m_entries.insert(key, std::move(e));
	m_fifoKeys.append(key);
	evictOverflow();
}

const PlanResultCache::FeasibleAxisEntry* PlanResultCache::fetchFeasibleAxis(const QString& instructionId,
																			const QString& fingerprint) const
{
	const auto it = m_feasibleAxis.find(makeKey(instructionId, fingerprint));
	return (it != m_feasibleAxis.end()) ? &(*it) : nullptr;
}

void PlanResultCache::storeFeasibleAxis(const QString& instructionId, const QString& fingerprint,
										const RobotInstruction::FeasibleMotionAxisConfigurationOptions& options,
										const QVector<double>& seedJointRad)
{
	const QString key = makeKey(instructionId, fingerprint);
	if (m_feasibleAxis.contains(key))
	{
		m_feasibleFifoKeys.removeAll(key);
	}
	FeasibleAxisEntry e;
	e.options = options;
	e.seedJointRad = seedJointRad;
	m_feasibleAxis.insert(key, std::move(e));
	m_feasibleFifoKeys.append(key);
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
	auto fit = m_feasibleAxis.begin();
	while (fit != m_feasibleAxis.end())
	{
		if (fit.key().startsWith(instructionId + QLatin1Char('|')))
		{
			m_feasibleFifoKeys.removeAll(fit.key());
			fit = m_feasibleAxis.erase(fit);
		}
		else
		{
			++fit;
		}
	}
}

void PlanResultCache::invalidateAll()
{
	m_entries.clear();
	m_fifoKeys.clear();
	m_feasibleAxis.clear();
	m_feasibleFifoKeys.clear();
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
