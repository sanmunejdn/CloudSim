#ifndef ROBOTURDF_SELFTEST_H
#define ROBOTURDF_SELFTEST_H

/// @file SelfTest.h
/// @brief RobotUrdf FK/J/DLS 自检（有限差分 + golden）

#include "robot_urdf_global.h"

#include <string>
#include <vector>

namespace UrdfRobotLoader
{
ROBOT_URDF_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_SELFTEST_H
