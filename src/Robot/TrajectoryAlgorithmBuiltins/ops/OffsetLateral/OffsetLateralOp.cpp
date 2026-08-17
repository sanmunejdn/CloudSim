/// @file OffsetLateralOp.cpp
/// @brief OffsetLateral 轨迹算子

// OffsetLateral 原子块：沿横向偏移路径点
#include "OffsetLateralOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind OffsetLateralOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::OffsetLateral;
}

const char* OffsetLateralOp::displayName(const bool chinese) const
{
	return chinese ? "横向偏移" : "OffsetLateral";
}

TrajectoryOpCapability OffsetLateralOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
OffsetLateralOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::OffsetLateral;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> OffsetLateralOp::paramFields() const
{
	return {
		doubleParamField("offset.lateralMm", "Lateral", "横向偏移", "mm", -1e5, 1e5, 0.01, 0.0, 0),
	};
}

bool OffsetLateralOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string OffsetLateralOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool OffsetLateralOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
								  RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
								  std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	offsetLateralUnifiedInScope(traj, op.scope, ctx.program, parsePathOffsetParams(op.params).lateralMm);
	return true;
}

} // namespace trajectory_algo
