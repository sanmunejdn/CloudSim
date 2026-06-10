// ExternalAxisSearch 原子块：搜索外部轴以满足可达性
#include "ExternalAxisSearchOp.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind ExternalAxisSearchOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ExternalAxisSearch;
}

const char* ExternalAxisSearchOp::displayName(const bool chinese) const
{
	return chinese ? "外部轴搜索" : "ExternalAxisSearch";
}

TrajectoryOpCapability ExternalAxisSearchOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor ExternalAxisSearchOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ExternalAxisSearch;
	op.scope = defaultScope;
	return op;
}

std::vector<TrajectoryOpParamField> ExternalAxisSearchOp::paramFields() const
{
	return {};
}

bool ExternalAxisSearchOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ExternalAxisSearchOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ExternalAxisSearchOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	(void)op;
	(void)ctx;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	externalAxisSearchUnified(traj);
	return true;
}

} // namespace trajectory_algo
