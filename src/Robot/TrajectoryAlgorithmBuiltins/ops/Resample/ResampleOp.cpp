/// @file ResampleOp.cpp
/// @brief ResampleOp 实现

// Resample 原子块：按步长重采样 UnifiedTrajectory 折线
#include "ResampleOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind ResampleOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Resample;
}

const char* ResampleOp::displayName(const bool chinese) const
{
	return chinese ? "重采样" : "Resample";
}

TrajectoryOpCapability ResampleOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ResampleOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Resample;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> ResampleOp::paramFields() const
{
	return {
		doubleParamField("resample.stepMm", "Step", "点距", "mm", -1e5, 1e5, 0.01, 5.0, 0),
	};
}

bool ResampleOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (parseResampleParams(op.params).stepMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "resample step must be positive";
		}
		return false;
	}
	return true;
}

std::string ResampleOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ResampleOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
							 RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
							 std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	resampleUnifiedTrajectoryInScope(traj, op.scope, ctx.program, parseResampleParams(op.params).stepMm);
	return true;
}

} // namespace trajectory_algo
