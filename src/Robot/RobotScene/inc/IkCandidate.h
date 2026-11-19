#ifndef ROBOTSCENE_IKCANDIDATE_H
#define ROBOTSCENE_IKCANDIDATE_H

/// @file IkCandidate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IK 候选一等公民（流水线入池 / Rank 共用）

#include "robot_scene_global.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
struct ROBOT_SCENE_API IkCandidate
{
	std::vector<double> q;
	double dist = 0.0;
	double wristJump = 0.0;
	double manip = 0.0;
	double collisionCost = 0.0;
	double softPen = 0.0;
	int softCfg = 0;
	int seedOrder = 0;
	int branchId = -1;
	std::string solverName;
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKCANDIDATE_H
