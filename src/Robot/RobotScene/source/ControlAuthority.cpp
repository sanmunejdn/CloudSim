/// @file ControlAuthority.cpp
/// @brief 控制权威表

#include "ControlAuthority.h"

#include <QList>

void ControlAuthorityMap::clear()
{
	m_map.clear();
}

void ControlAuthorityMap::setAuthority(int robotInstanceIndex, ControlAuthorityKind kind)
{
	if (robotInstanceIndex < 0)
	{
		return;
	}
	if (kind == ControlAuthorityKind::Kernel)
	{
		m_map.remove(robotInstanceIndex);
		return;
	}
	m_map.insert(robotInstanceIndex, kind);
}

ControlAuthorityKind ControlAuthorityMap::authority(int robotInstanceIndex) const
{
	return m_map.value(robotInstanceIndex, ControlAuthorityKind::Kernel);
}

bool ControlAuthorityMap::isExternal(int robotInstanceIndex) const
{
	return authority(robotInstanceIndex) == ControlAuthorityKind::External;
}

QList<int> ControlAuthorityMap::externalInstances() const
{
	return m_map.keys();
}
