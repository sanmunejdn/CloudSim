/// @file SyncBarrierRegistry.cpp
/// @brief SyncBarrier 注册表

#include "SyncBarrierRegistry.h"

void SyncBarrierRegistry::clear()
{
	m_barriers.clear();
	m_waitingByLane.clear();
}

bool SyncBarrierRegistry::arrive(const QString& name, const SimulationParticipantId& id,
								 const QVector<SimulationParticipantId>& expected, double simTimeSec, bool* timedOut)
{
	if (timedOut)
	{
		*timedOut = false;
	}
	if (name.isEmpty() || !id.isValid())
	{
		return true;
	}
	BarrierState& st = m_barriers[name];
	if (st.firstArriveSimTime < 0.0)
	{
		st.firstArriveSimTime = simTimeSec;
	}
	st.arrivedKeys.insert(id.key());
	m_waitingByLane.insert(id.key(), name);

	if (m_timeoutSec > 0.0 && (simTimeSec - st.firstArriveSimTime) > m_timeoutSec)
	{
		if (timedOut)
		{
			*timedOut = true;
		}
		m_waitingByLane.remove(id.key());
		return false;
	}

	for (const SimulationParticipantId& e : expected)
	{
		if (!st.arrivedKeys.contains(e.key()))
		{
			return false;
		}
	}
	// 全员到齐：清等待
	for (const SimulationParticipantId& e : expected)
	{
		m_waitingByLane.remove(e.key());
	}
	m_barriers.remove(name);
	return true;
}

bool SyncBarrierRegistry::isWaiting(const SimulationParticipantId& id, QString* outName) const
{
	const auto it = m_waitingByLane.constFind(id.key());
	if (it == m_waitingByLane.cend())
	{
		return false;
	}
	if (outName)
	{
		*outName = it.value();
	}
	return true;
}

void SyncBarrierRegistry::leave(const SimulationParticipantId& id)
{
	m_waitingByLane.remove(id.key());
}
