#ifndef ROBOTSCENE_IKSEEDPROVIDER_H
#define ROBOTSCENE_IKSEEDPROVIDER_H

/// @file IkSeedProvider.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轴配置感知的 IK 种子扩展（expandPoseIkSeeds + 肘腕臂扰动）

#include "robot_scene_global.h"

#include "RobotInstructionAxisConfiguration.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
ROBOT_SCENE_API int ikJointIndexByNameHint(const std::vector<std::string>& jointNames, const char* hint,
										   int fallbackIndex);

ROBOT_SCENE_API std::vector<std::vector<double>> buildIkSeedVariants(
	const std::vector<double>& qSeed, const std::vector<std::string>& jointNames,
	const MotionAxisConfiguration* axisCfg = nullptr, bool sphericalWrist = true, int elbowJointIndexOverride = -1,
	int wristJointIndexOverride = -1);

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKSEEDPROVIDER_H
