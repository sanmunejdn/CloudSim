#ifndef ROBOTSCENE_ROBOTKINEMATICMODELREGISTRATION_H
#define ROBOTSCENE_ROBOTKINEMATICMODELREGISTRATION_H

/// @file RobotKinematicModelRegistration.h
/// @brief URDF 实例创建时注册 Composite（臂 + 外轴）到 KinematicModelRegistry

#include "robot_scene_global.h"

#include <QString>

class IRobotSimulationDocument;

namespace RobotKinematicModelRegistration
{
ROBOT_SCENE_API bool registerRobotInstance(IRobotSimulationDocument* doc, int instanceIndex,
										   const QString& sceneRootBackendId);
}

#endif // ROBOTSCENE_ROBOTKINEMATICMODELREGISTRATION_H
