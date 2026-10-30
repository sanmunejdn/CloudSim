/// @file InsertMidOp.cpp
/// @brief InsertMid 轨迹算子

// InsertMid：每段中点插值加密
#include "InsertMidOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind InsertMidOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::InsertMid;
}

const char* InsertMidOp::displayName(const bool chinese) const
{
	return chinese ? "段中插点" : "Insert Midpoints";
}

TrajectoryOpCapability InsertMidOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
InsertMidOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::InsertMid;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> InsertMidOp::paramFields() const
{
	return {
		intParamField("insert.passes", "Passes", "加密遍数", 0, 999999, 1, 0),
	};
}

bool InsertMidOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (trajectoryParamInt(op.params, "insert.passes", 1) < 1)
	{
		if (errMsg)
		{
			*errMsg = "insert passes must be >= 1";
		}
		return false;
	}
	return true;
}

std::string InsertMidOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool InsertMidOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const int passes = trajectoryParamInt(op.params, "insert.passes", 1);
	insertMidpointsUnifiedInScope(traj, op.scope, ctx.program, passes);
	return true;
}

} // namespace trajectory_algo
