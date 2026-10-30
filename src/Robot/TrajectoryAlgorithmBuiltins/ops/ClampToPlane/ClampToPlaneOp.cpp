/// @file ClampToPlaneOp.cpp
/// @brief ClampToPlane 轨迹算子

// ClampToPlane：将坐标轴锁到给定值
#include "ClampToPlaneOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind ClampToPlaneOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ClampToPlane;
}

const char* ClampToPlaneOp::displayName(const bool chinese) const
{
	return chinese ? "贴平面" : "Clamp To Plane";
}

TrajectoryOpCapability ClampToPlaneOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ClampToPlaneOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ClampToPlane;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> ClampToPlaneOp::paramFields() const
{
	return {
		enumParamField("clamp.axis", "Axis", "锁定轴", {"0", "1", "2"}, {"X", "Y", "Z"}, {"X", "Y", "Z"}, 2, 0),
		doubleParamField("clamp.valueMm", "Value", "目标值", "mm", -1e5, 1e5, 0.01, 0.0, 1),
	};
}

bool ClampToPlaneOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ClampToPlaneOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ClampToPlaneOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const int axis = trajectoryParamInt(op.params, "clamp.axis", 2);
	const double valueMm = trajectoryParamDouble(op.params, "clamp.valueMm", 0.0);
	clampToPlaneUnifiedInScope(traj, op.scope, ctx.program, axis, valueMm);
	return true;
}

} // namespace trajectory_algo
