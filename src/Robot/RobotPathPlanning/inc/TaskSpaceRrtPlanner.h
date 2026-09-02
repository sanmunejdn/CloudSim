#ifndef ROBOTPATHPLANNING_TASKSPACERRTPLANNER_H
#define ROBOTPATHPLANNING_TASKSPACERRTPLANNER_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "CollisionValidity.h"
#include "RobotPathPlanning.h"

namespace robot_path
{
namespace detail
{

/// SE(3) 任务空间 RRTConnect + 逐点 IK + 关节碰撞门控
bool planTaskSpaceRrt(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ,
					  PathResult& out);

} // namespace detail
} // namespace robot_path

#endif
