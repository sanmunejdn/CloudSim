/// @file SpikeFilterOp.cpp
/// @brief SpikeFilter 轨迹算子

// SpikeFilter：去掉相对弦线过长的尖点
#include "SpikeFilterOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

#include <cmath>

namespace trajectory_algo
{
RobotInstruction::TrajectoryOpKind SpikeFilterOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::SpikeFilter;
}

const char* SpikeFilterOp::displayName(const bool chinese) const
{
	return chinese ? "尖点滤波" : "Spike Filter";
}

TrajectoryOpCapability SpikeFilterOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
SpikeFilterOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::SpikeFilter;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> SpikeFilterOp::paramFields() const
{
	return {
		doubleParamField("spike.maxDetourRatio", "Max detour ratio", "绕行比阈值", "", -1e5, 1e5, 0.01, 1.5, 0),
	};
}

bool SpikeFilterOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (trajectoryParamDouble(op.params, "spike.maxDetourRatio", 1.5) < 1.0)
	{
		if (errMsg)
		{
			*errMsg = "detour ratio must be >= 1";
		}
		return false;
	}
	return true;
}

std::string SpikeFilterOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool SpikeFilterOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const double ratio = trajectoryParamDouble(op.params, "spike.maxDetourRatio", 1.5);
	spikeFilterUnifiedInScope(traj, op.scope, ctx.program, ratio);
	return true;
}

} // namespace trajectory_algo
