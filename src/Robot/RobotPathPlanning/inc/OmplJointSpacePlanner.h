#ifndef ROBOTPATHPLANNING_OMPLJOINTSPACEPLANNER_H
#define ROBOTPATHPLANNING_OMPLJOINTSPACEPLANNER_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "CollisionValidity.h"
#include "RobotPathPlanning.h"

namespace robot_path
{
namespace detail
{

#if defined(CLOUDSIM_HAS_OMPL)
bool planJointSpaceOmpl(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ, PathResult& out);
#endif

} // namespace detail
} // namespace robot_path

#endif
