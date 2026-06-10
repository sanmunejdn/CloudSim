// ReachabilityFilter 原子块：剔除不可达路径点
#include "ReachabilityFilterOp.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind ReachabilityFilterOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ReachabilityFilter;
}

const char* ReachabilityFilterOp::displayName(const bool chinese) const
{
	return chinese ? "可达性过滤" : "ReachabilityFilter";
}

TrajectoryOpCapability ReachabilityFilterOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor ReachabilityFilterOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ReachabilityFilter;
	op.scope = defaultScope;
	return op;
}

std::vector<TrajectoryOpParamField> ReachabilityFilterOp::paramFields() const
{
	return {};
}

bool ReachabilityFilterOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ReachabilityFilterOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ReachabilityFilterOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	reachabilityFilterUnifiedInScope(traj, op.scope, ctx.program);
	return true;
}

} // namespace trajectory_algo
