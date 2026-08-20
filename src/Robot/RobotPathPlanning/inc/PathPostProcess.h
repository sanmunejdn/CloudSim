#ifndef ROBOTPATHPLANNING_PATHPOSTPROCESS_H
#define ROBOTPATHPLANNING_PATHPOSTPROCESS_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include "CollisionValidity.h"
#include "RobotPathPlanning.h"

namespace robot_path
{
namespace detail
{

void fillTcpPosesFromJoints(const PlanRequest& req, PathResult& io);

/// 按关节步长加密路径（写入程序中间点）
void densifyJointPath(PathResult& io, double maxStepRad);

/// 关节路径捷径缩短（确定性，同输入同输出）
void shortcutJointPath(const PlanRequest& req, const JointLimits& lim, PathResult& io);

void computePathMetrics(PathResult& io);

} // namespace detail
} // namespace robot_path

#endif
