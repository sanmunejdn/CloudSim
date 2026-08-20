#ifndef ROBOTPATHPLANNING_JOINTSPACERRTPLANNER_H
#define ROBOTPATHPLANNING_JOINTSPACERRTPLANNER_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "CollisionValidity.h"
#include "RobotPathPlanning.h"

namespace robot_path
{
namespace detail
{

/// 关节空间 RRTConnect / RRT*；OMPL 未链入时使用，API 与 OmplJointSpacePlanner 一致
bool planJointSpaceRrt(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ,
					   PathResult& out);

} // namespace detail
} // namespace robot_path

#endif
