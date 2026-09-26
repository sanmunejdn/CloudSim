#ifndef ROBOTPATHPLANNING_SELFTEST_H
#define ROBOTPATHPLANNING_SELFTEST_H

/// @file SelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotPathPlanning 自检入口（避免 SelfTestRunner 拉取重头文件）

#include "robot_path_planning_global.h"

#include <string>
#include <vector>

namespace robot_path
{
ROBOT_PATH_PLANNING_API bool runSelfTest(std::vector<std::string>& failures);
}

#endif // ROBOTPATHPLANNING_SELFTEST_H
