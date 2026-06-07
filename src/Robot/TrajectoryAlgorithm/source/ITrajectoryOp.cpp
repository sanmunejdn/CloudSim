// processPath 默认空实现，仅 Pose 变换块可不覆写
#include "ITrajectoryOp.h"

namespace trajectory_algo
{

bool ITrajectoryOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg) const
{
	(void)op;
	(void)traj;
	(void)errMsg;
	return false;
}

} // namespace trajectory_algo
