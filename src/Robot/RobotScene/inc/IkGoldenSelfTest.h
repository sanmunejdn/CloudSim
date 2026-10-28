#ifndef ROBOTSCENE_IKGOLDENSELFTEST_H
#define ROBOTSCENE_IKGOLDENSELFTEST_H

/// @file IkGoldenSelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机型包级 IK golden（无整臂仿真；校验构型/支编码约定）

#include "robot_scene_global.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
/// 加载 JSON；path 空则用内置 synthetic 路径（相对本编译单元）
ROBOT_SCENE_API bool runIkGoldenSelfTest(std::vector<std::string>* failures = nullptr,
										 const std::string& jsonPathUtf8 = {});

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKGOLDENSELFTEST_H
