#ifndef ROBOTSCENE_EVENTLOGICAUTHORITY_H
#define ROBOTSCENE_EVENTLOGICAUTHORITY_H

/// @file EventLogicAuthority.h
/// @brief EventBased 下逻辑权威：本地薄 CEE vs 外接 PLC

#include "robot_scene_global.h"

#include <QString>

enum class ROBOT_SCENE_API EventLogicAuthority
{
	LocalCee = 0,
	ExternalPlc
};

inline QString eventLogicAuthorityToString(EventLogicAuthority a)
{
	return a == EventLogicAuthority::ExternalPlc ? QStringLiteral("externalPlc") : QStringLiteral("localCee");
}

inline EventLogicAuthority eventLogicAuthorityFromString(const QString& s)
{
	if (s.compare(QStringLiteral("externalPlc"), Qt::CaseInsensitive) == 0)
	{
		return EventLogicAuthority::ExternalPlc;
	}
	return EventLogicAuthority::LocalCee;
}

#endif // ROBOTSCENE_EVENTLOGICAUTHORITY_H
