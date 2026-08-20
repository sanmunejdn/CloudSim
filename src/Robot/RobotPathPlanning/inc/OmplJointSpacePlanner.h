#ifndef ROBOTPATHPLANNING_OMPLJOINTSPACEPLANNER_H
#define ROBOTPATHPLANNING_OMPLJOINTSPACEPLANNER_H

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
