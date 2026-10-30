/// @file ReversePathOp.cpp
/// @brief ReversePath 轨迹算子

// ReversePath：反转作用域内点序
#include "ReversePathOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind ReversePathOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ReversePath;
}

const char* ReversePathOp::displayName(const bool chinese) const
{
	return chinese ? "路径反向" : "Reverse Path";
}

TrajectoryOpCapability ReversePathOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ReversePathOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ReversePath;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> ReversePathOp::paramFields() const
{
	return {};
}

bool ReversePathOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ReversePathOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ReversePathOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	reversePathUnifiedInScope(traj, op.scope, ctx.program);
	return true;
}

} // namespace trajectory_algo
