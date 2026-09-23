#ifndef ROBOTSCENE_URDFROBOTKINEMATICMODELSINK_H
#define ROBOTSCENE_URDFROBOTKINEMATICMODELSINK_H

/// @file UrdfRobotKinematicModelSink.h
/// @brief UrdfRobotKinematicModelSink 接口

#include "robot_scene_global.h"

#include "RobotKinematicApplyContext.h"
#include "UrdfRobotKinematicModel.h"

#include <QVector>
#include <vector>

namespace UrdfRobotKinematicModelSink
{
ROBOT_SCENE_API bool applyToSink(const UrdfRobotKinematicModel::Model& model,
								 const RobotKinematicApplyContext::Context& ctx, const std::vector<double>& localArmQ,
								 QVector<double>& aggregatedAnglesRad);
}

#endif // ROBOTSCENE_URDFROBOTKINEMATICMODELSINK_H
