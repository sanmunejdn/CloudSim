// processPath 默认空实现，仅 Pose 变换块可不覆写
#include "ITrajectoryOp.h"

namespace trajectory_algo
{

bool ITrajectoryOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	(void)op;
	(void)traj;
	(void)ctx;
	(void)errMsg;
	return false;
}

} // namespace trajectory_algo
