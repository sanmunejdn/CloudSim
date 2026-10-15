/// @file IoPhase.cpp
/// @brief IO 相位

#include "IoPhase.h"

#include "IIoSignalNetwork.h"

LatchedIoSink* IoPhase::ensureLatchedSink(const QString& ownerId)
{
	if (ownerId.isEmpty())
	{
		return nullptr;
	}
	auto it = m_latched.find(ownerId);
	if (it != m_latched.end())
	{
		if (m_network)
		{
			it.value()->setBackend(m_network->sink(ownerId));
		}
		return it.value().get();
	}
	IRobotIoSink* backend = m_network ? m_network->sink(ownerId) : nullptr;
	auto sink = std::make_shared<LatchedIoSink>(backend);
	m_latched.insert(ownerId, sink);
	return sink.get();
}

LatchedIoSink* IoPhase::latchedSink(const QString& ownerId) const
{
	const auto it = m_latched.constFind(ownerId);
	return it == m_latched.cend() ? nullptr : it.value().get();
}

void IoPhase::clear()
{
	m_latched.clear();
}

void IoPhase::latchAfterPropagate(const QHash<QString, QList<int>>& portsByOwner)
{
	if (m_network)
	{
		m_network->propagateAll();
	}
	for (auto it = portsByOwner.constBegin(); it != portsByOwner.constEnd(); ++it)
	{
		LatchedIoSink* sink = ensureLatchedSink(it.key());
		if (sink)
		{
			sink->capturePortsToNext(it.value());
		}
	}
}

void IoPhase::swapAll()
{
	for (auto it = m_latched.begin(); it != m_latched.end(); ++it)
	{
		it.value()->swapNextToCurrent();
	}
}
