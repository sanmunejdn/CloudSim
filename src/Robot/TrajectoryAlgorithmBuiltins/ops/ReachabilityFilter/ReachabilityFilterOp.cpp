/// @file ReachabilityFilterOp.cpp
/// @brief ReachabilityFilter 轨迹算子

// 对流水线当前位姿做真 IK 探测，写 reachable（下游 emit 跳过不可达点）
#include "ReachabilityFilterOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

#include <cstdio>

namespace trajectory_algo
{
namespace
{
TrajectoryOpParamField boolParamField(const std::string& key, const std::string& labelEn, const std::string& labelZh,
									  const bool defaultValue, const int order)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Bool;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.defaultBool = defaultValue;
	field.order = order;
	return field;
}
} // namespace

RobotInstruction::TrajectoryOpKind ReachabilityFilterOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ReachabilityFilter;
}

const char* ReachabilityFilterOp::displayName(const bool chinese) const
{
	return chinese ? "可达性过滤" : "ReachabilityFilter";
}

TrajectoryOpCapability ReachabilityFilterOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ReachabilityFilterOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ReachabilityFilter;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> ReachabilityFilterOp::paramFields() const
{
	return {
		boolParamField("reachability.useOrientation", "Use orientation", "约束姿态", true, 0),
		doubleParamField("reachability.residualTolMm", "Position tol", "位置容差", "mm", 0.1, 100.0, 0.1, 5.0, 1),
	};
}

bool ReachabilityFilterOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ReachabilityFilterOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op,
												const bool chinese) const
{
	const bool useOri = trajectoryParamBool(op.params, "reachability.useOrientation", true);
	const double tol = trajectoryParamDouble(op.params, "reachability.residualTolMm", 5.0);
	char buffer[128];
	if (chinese)
	{
		std::snprintf(buffer, sizeof(buffer), "可达性过滤 | 姿态=%s 容差=%.1fmm", useOri ? "开" : "关", tol);
	}
	else
	{
		std::snprintf(buffer, sizeof(buffer), "ReachabilityFilter | ori=%s tol=%.1fmm", useOri ? "on" : "off", tol);
	}
	return buffer;
}

bool ReachabilityFilterOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
									   RobotInstruction::UnifiedTrajectory& traj,
									   const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const
{
	if (traj.points.empty())
	{
		return false;
	}
	const bool useOrientation = trajectoryParamBool(op.params, "reachability.useOrientation", true);
	const double residualTolMm = trajectoryParamDouble(op.params, "reachability.residualTolMm", 5.0);
	return reachabilityFilterUnifiedInScope(traj, op.scope, ctx.program, ctx, useOrientation, residualTolMm, errMsg);
}

} // namespace trajectory_algo
