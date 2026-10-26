#ifndef ROBOTSCENE_ICEEACTIONHANDLER_H
#define ROBOTSCENE_ICEEACTIONHANDLER_H

/// @file ICeeActionHandler.h
/// @brief CEE 中需 Controller 侧完成的动作（启姿态 / 启机器人程序）

#include "robot_scene_global.h"

#include <QString>

class ROBOT_SCENE_API ICeeActionHandler
{
public:
	virtual ~ICeeActionHandler() = default;

	virtual bool onGoNamedPose(const QString& deviceId, const QString& poseId, double durationSec, QString* err) = 0;
	virtual bool onStartRobotProgram(int instanceIndex, const QString& programId, QString* err) = 0;
};

#endif // ROBOTSCENE_ICEEACTIONHANDLER_H
