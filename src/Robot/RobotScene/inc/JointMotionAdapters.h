#ifndef ROBOTSCENE_JOINTMOTIONADAPTERS_H
#define ROBOTSCENE_JOINTMOTIONADAPTERS_H

#include "CustomDeviceBackendData.h"
#include "RobotExternalAxes.h"
#include "robot_scene_global.h"

#include "JointMotion1D.h"

namespace JointMotionAdapters
{
ROBOT_SCENE_API kinematic_core::JointMotion1D fromCustomDeviceAxisConfig(const CustomDeviceAxisConfig& in);
ROBOT_SCENE_API kinematic_core::JointMotion1D fromRobotExternalAxisConfig(const RobotExternal::RobotExternalAxisConfig& in);
}

#endif // ROBOTSCENE_JOINTMOTIONADAPTERS_H
