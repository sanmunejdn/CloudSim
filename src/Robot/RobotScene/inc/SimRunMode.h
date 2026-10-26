#ifndef ROBOTSCENE_SIMRUNMODE_H
#define ROBOTSCENE_SIMRUNMODE_H

/// @file SimRunMode.h
/// @brief 仿真运行模式：时间回放 vs 事件（薄 CEE）

#include "robot_scene_global.h"

#include <QString>

enum class ROBOT_SCENE_API SimRunMode
{
	TimeBased = 0,
	EventBased
};

inline QString simRunModeToString(SimRunMode mode)
{
	return mode == SimRunMode::EventBased ? QStringLiteral("eventBased") : QStringLiteral("timeBased");
}

inline SimRunMode simRunModeFromString(const QString& s)
{
	if (s.compare(QStringLiteral("eventBased"), Qt::CaseInsensitive) == 0)
	{
		return SimRunMode::EventBased;
	}
	return SimRunMode::TimeBased;
}

#endif // ROBOTSCENE_SIMRUNMODE_H
