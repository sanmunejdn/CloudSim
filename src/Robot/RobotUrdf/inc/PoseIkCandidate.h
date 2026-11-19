#ifndef ROBOTURDF_POSEIKCANDIDATE_H
#define ROBOTURDF_POSEIKCANDIDATE_H

/// @file PoseIkCandidate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 跨求解器位姿 IK 候选

#include "robot_urdf_global.h"

#include "UrdfIkSolverOptions.h"

#include <vector>

namespace UrdfRobotLoader
{
struct ROBOT_URDF_API PoseIkCandidate
{
	std::vector<double> q;
	int branchId = -1;
	IkConvergenceStatus status = IkConvergenceStatus::Failed;
	const char* solverName = "";
};

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_POSEIKCANDIDATE_H
