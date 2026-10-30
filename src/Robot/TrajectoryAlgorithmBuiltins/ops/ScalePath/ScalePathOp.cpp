/// @file ScalePathOp.cpp
/// @brief ScalePath 轨迹算子

// ScalePath：相对作用域首点缩放位置
#include "ScalePathOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

#include <cmath>

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind ScalePathOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ScalePath;
}

const char* ScalePathOp::displayName(const bool chinese) const
{
	return chinese ? "路径缩放" : "Scale Path";
}

TrajectoryOpCapability ScalePathOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ScalePathOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ScalePath;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> ScalePathOp::paramFields() const
{
	return {
		doubleParamField("scale.factor", "Factor", "比例", "", -1e5, 1e5, 0.01, 1.0, 0),
	};
}

bool ScalePathOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (std::abs(trajectoryParamDouble(op.params, "scale.factor", 1.0)) < 1e-9)
	{
		if (errMsg)
		{
			*errMsg = "scale factor must be non-zero";
		}
		return false;
	}
	return true;
}

std::string ScalePathOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ScalePathOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const double factor = trajectoryParamDouble(op.params, "scale.factor", 1.0);
	scalePathUnifiedInScope(traj, op.scope, ctx.program, factor);
	return true;
}

} // namespace trajectory_algo
