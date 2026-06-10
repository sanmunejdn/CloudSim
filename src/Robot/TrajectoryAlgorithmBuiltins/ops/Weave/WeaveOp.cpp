// Weave 原子块：在路径上叠加摆动
#include "WeaveOp.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind WeaveOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Weave;
}

const char* WeaveOp::displayName(const bool chinese) const
{
	return chinese ? "摆动" : "Weave";
}

TrajectoryOpCapability WeaveOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor WeaveOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Weave;
	op.scope = defaultScope;
	return op;
}

std::vector<TrajectoryOpParamField> WeaveOp::paramFields() const
{
	return {
		doubleParamField("weave.amplitudeMm", "Amplitude", "振幅", "mm", -1e5, 1e5, 0.01, 2.0, 0),
		doubleParamField("weave.periodMm", "Period", "周期", "mm", -1e5, 1e5, 0.01, 10.0, 1),
	};
}

bool WeaveOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string WeaveOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool WeaveOp::processPath(
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
	weaveUnifiedInScope(
		traj,
		op.scope,
		ctx.program,
		op.weave.amplitudeMm,
		op.weave.periodMm);
	return true;
}

} // namespace trajectory_algo
