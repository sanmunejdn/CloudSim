#ifndef ROBOTSCENE_JOINTMOTIONADAPTERS_H
#define ROBOTSCENE_JOINTMOTIONADAPTERS_H

/// @file JointMotionAdapters.h
/// @brief JointMotionAdapters 接口

#include "robot_scene_global.h"

#include "CustomDeviceBackendData.h"
#include "JointMotion1D.h"
#include "RobotExternalAxes.h"

namespace JointMotionAdapters
{
ROBOT_SCENE_API kinematic_core::JointMotion1D fromCustomDeviceAxisConfig(const CustomDeviceAxisConfig& in);
ROBOT_SCENE_API kinematic_core::JointMotion1D
fromRobotExternalAxisConfig(const RobotExternal::RobotExternalAxisConfig& in);
} // namespace JointMotionAdapters

#endif // ROBOTSCENE_JOINTMOTIONADAPTERS_H
