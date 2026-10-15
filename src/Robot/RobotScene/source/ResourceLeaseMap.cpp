/// @file ResourceLeaseMap.cpp
/// @brief 资源租约表

#include "ResourceLeaseMap.h"

void ResourceLeaseMap::clear()
{
	m_holders.clear();
}

bool ResourceLeaseMap::tryAcquire(const QString& resourceKey, const SimulationParticipantId& owner, QString* errOut)
{
	if (resourceKey.isEmpty() || !owner.isValid())
	{
		if (errOut)
		{
			*errOut = QStringLiteral("Invalid lease acquire.");
		}
		return false;
	}
	const auto it = m_holders.constFind(resourceKey);
	if (it == m_holders.cend())
	{
		m_holders.insert(resourceKey, owner);
		return true;
	}
	if (it.value() == owner)
	{
		return true;
	}
	if (errOut)
	{
		*errOut = QStringLiteral("Resource lease conflict: %1 held by %2, requested by %3")
					  .arg(resourceKey, it.value().key(), owner.key());
	}
	return false;
}

void ResourceLeaseMap::release(const QString& resourceKey, const SimulationParticipantId& owner)
{
	const auto it = m_holders.constFind(resourceKey);
	if (it != m_holders.cend() && it.value() == owner)
	{
		m_holders.erase(it);
	}
}

void ResourceLeaseMap::releaseAll(const SimulationParticipantId& owner)
{
	for (auto it = m_holders.begin(); it != m_holders.end();)
	{
		if (it.value() == owner)
		{
			it = m_holders.erase(it);
		}
		else
		{
			++it;
		}
	}
}

SimulationParticipantId ResourceLeaseMap::ownerOf(const QString& resourceKey) const
{
	return m_holders.value(resourceKey);
}

bool ResourceLeaseMap::isHeldBy(const QString& resourceKey, const SimulationParticipantId& owner) const
{
	const auto it = m_holders.constFind(resourceKey);
	return it != m_holders.cend() && it.value() == owner;
}
