/// @file TrimPathOp.cpp
/// @brief TrimPath 轨迹算子

// TrimPath：按弧长去掉首尾
#include "TrimPathOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

#include <cmath>

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind TrimPathOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::TrimPath;
}

const char* TrimPathOp::displayName(const bool chinese) const
{
	return chinese ? "弧长裁剪" : "Trim Path";
}

TrajectoryOpCapability TrimPathOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
TrimPathOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::TrimPath;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> TrimPathOp::paramFields() const
{
	return {
		doubleParamField("trim.startMm", "Trim start", "起点裁剪", "mm", -1e5, 1e5, 0.01, 0.0, 0),
		doubleParamField("trim.endMm", "Trim end", "终点裁剪", "mm", -1e5, 1e5, 0.01, 0.0, 1),
	};
}

bool TrimPathOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (trajectoryParamDouble(op.params, "trim.startMm", 0.0) < 0.0 ||
		trajectoryParamDouble(op.params, "trim.endMm", 0.0) < 0.0)
	{
		if (errMsg)
		{
			*errMsg = "trim lengths must be >= 0";
		}
		return false;
	}
	return true;
}

std::string TrimPathOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool TrimPathOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const double startMm = trajectoryParamDouble(op.params, "trim.startMm", 0.0);
	const double endMm = trajectoryParamDouble(op.params, "trim.endMm", 0.0);
	trimPathUnifiedInScope(traj, op.scope, ctx.program, startMm, endMm);
	return true;
}

} // namespace trajectory_algo
