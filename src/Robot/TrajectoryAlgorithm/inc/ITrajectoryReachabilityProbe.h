#ifndef TRAJECTORYALGORITHM_ITRAJECTORYREACHABILITYPROBE_H
#define TRAJECTORYALGORITHM_ITRAJECTORYREACHABILITYPROBE_H

/// @file ITrajectoryReachabilityProbe.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轨迹点 IK 可达探测：由 RobotScene/UI 注入；未注入时 ReachabilityFilter 失败

#include "trajectory_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace RobotInstruction
{
struct UnifiedTrajectory;
}

namespace trajectory_algo
{
class TRAJECTORY_ALGORITHM_API ITrajectoryReachabilityProbe
{
public:
	virtual ~ITrajectoryReachabilityProbe() = default;

	/// 对 indices 内点写 reachable；indices 空则全点；链式滚动种子
	virtual bool probe(RobotInstruction::UnifiedTrajectory& traj, const std::vector<std::size_t>& indices,
					   bool useOrientation, double residualTolMm, std::string* errMsg) const = 0;
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_ITRAJECTORYREACHABILITYPROBE_H
