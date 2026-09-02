#ifndef ROBOTPATHPLANNING_JOINTSPACEDIJKSTRAPLANNER_H
#define ROBOTPATHPLANNING_JOINTSPACEDIJKSTRAPLANNER_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "CollisionValidity.h"
#include "RobotPathPlanning.h"

namespace robot_path
{
namespace detail
{

/// 关节空间均匀网格 + Dijkstra 最短路径（稀疏展开，不预建全维网格）
bool planJointSpaceDijkstra(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ,
							PathResult& out);

} // namespace detail
} // namespace robot_path

#endif
