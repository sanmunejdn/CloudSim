/// @file SpeedRampOp.cpp
/// @brief SpeedRamp 轨迹算子

// SpeedRamp：作用域内速度线性插值
#include "SpeedRampOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind SpeedRampOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::SpeedRamp;
}

const char* SpeedRampOp::displayName(const bool chinese) const
{
	return chinese ? "速度坡道" : "Speed Ramp";
}

TrajectoryOpCapability SpeedRampOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
SpeedRampOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::SpeedRamp;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> SpeedRampOp::paramFields() const
{
	return {
		doubleParamField("speed.startMmPerSec", "Start speed", "起点速度", "mm/s", -1e5, 1e5, 0.01, 50.0, 0),
		doubleParamField("speed.endMmPerSec", "End speed", "终点速度", "mm/s", -1e5, 1e5, 0.01, 100.0, 1),
	};
}

bool SpeedRampOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string SpeedRampOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool SpeedRampOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const double v0 = trajectoryParamDouble(op.params, "speed.startMmPerSec", 50.0);
	const double v1 = trajectoryParamDouble(op.params, "speed.endMmPerSec", 100.0);
	speedRampUnifiedInScope(traj, op.scope, ctx.program, v0, v1);
	return true;
}

} // namespace trajectory_algo
