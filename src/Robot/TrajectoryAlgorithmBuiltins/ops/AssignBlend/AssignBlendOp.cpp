// AssignBlend 原子块：为路径点写入 blend 半径
#include "AssignBlendOp.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind AssignBlendOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::AssignBlend;
}

const char* AssignBlendOp::displayName(const bool chinese) const
{
	return chinese ? "过渡半径" : "AssignBlend";
}

TrajectoryOpCapability AssignBlendOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor AssignBlendOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::AssignBlend;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> AssignBlendOp::paramFields() const
{
	return {
		doubleParamField("assign.blendRadiusMm", "Blend", "过渡半径", "mm", -1e5, 1e5, 0.01, 2.0, 0),
	};
}

bool AssignBlendOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string AssignBlendOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool AssignBlendOp::processPath(
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
	assignBlendUnifiedInScope(
		traj,
		op.scope,
		ctx.program,
		parseAssignMotionParams(op.params).blendRadiusMm);
	return true;
}

} // namespace trajectory_algo
