#ifndef ROBOTURDF_SELFTEST_H
#define ROBOTURDF_SELFTEST_H

/// @file SelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotUrdf FK/J/DLS 自检（有限差分 + golden）

#include "robot_urdf_global.h"

#include <string>
#include <vector>

namespace UrdfRobotLoader
{
ROBOT_URDF_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_SELFTEST_H
