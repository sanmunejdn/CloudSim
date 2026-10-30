/// @file ShiftStartOp.cpp
/// @brief ShiftStart 轨迹算子

// ShiftStart：闭合路径旋转起点
#include "ShiftStartOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind ShiftStartOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ShiftStart;
}

const char* ShiftStartOp::displayName(const bool chinese) const
{
	return chinese ? "闭环起点平移" : "Shift Start";
}

TrajectoryOpCapability ShiftStartOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ShiftStartOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ShiftStart;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> ShiftStartOp::paramFields() const
{
	return {
		intParamField("shift.startIndex1", "New start index", "新起点序号", 0, 999999, 1, 0),
	};
}

bool ShiftStartOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ShiftStartOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ShiftStartOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const int start1 = trajectoryParamInt(op.params, "shift.startIndex1", 1);
	shiftStartUnifiedInScope(traj, op.scope, ctx.program, start1);
	return true;
}

} // namespace trajectory_algo
