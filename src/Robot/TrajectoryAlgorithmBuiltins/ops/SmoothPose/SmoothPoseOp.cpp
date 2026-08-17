/// @file SmoothPoseOp.cpp
/// @brief SmoothPose 轨迹算子

// SmoothPose 原子块：平滑路径点位姿
#include "SmoothPoseOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind SmoothPoseOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::SmoothPose;
}

const char* SmoothPoseOp::displayName(const bool chinese) const
{
	return chinese ? "姿态平滑" : "SmoothPose";
}

TrajectoryOpCapability SmoothPoseOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
SmoothPoseOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::SmoothPose;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> SmoothPoseOp::paramFields() const
{
	return {};
}

bool SmoothPoseOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string SmoothPoseOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool SmoothPoseOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
							   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
							   std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	smoothPoseUnifiedInScope(traj, op.scope, ctx.program);
	return true;
}

} // namespace trajectory_algo
