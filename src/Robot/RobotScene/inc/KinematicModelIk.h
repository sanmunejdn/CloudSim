#ifndef ROBOTSCENE_KINEMATICMODELIK_H
#define ROBOTSCENE_KINEMATICMODELIK_H

/// @file KinematicModelIk.h
/// @brief Registry 编排示教 IK（臂走 KinematicCore，外轴复用 TeachIk 策略）

#include "RobotTeachIk.h"
#include "robot_scene_global.h"

#include <string>

namespace KinematicModelIk
{
ROBOT_SCENE_API RobotTeachIk::TeachIkResult solveTeachPose(const std::string& registryKey,
															 RobotTeachIk::TeachIkContext ctx);
}

#endif // ROBOTSCENE_KINEMATICMODELIK_H
