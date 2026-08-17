/// @file ExternalAxisSearchOp.cpp
/// @brief ExternalAxisSearch 轨迹算子

#include "ExternalAxisSearchOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

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

RobotInstruction::TrajectoryOpKind ExternalAxisSearchOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ExternalAxisSearch;
}

const char* ExternalAxisSearchOp::displayName(const bool chinese) const
{
	return chinese ? "外部轴搜索" : "ExternalAxisSearch";
}

TrajectoryOpCapability ExternalAxisSearchOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
ExternalAxisSearchOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ExternalAxisSearch;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);

	return op;
}

std::vector<TrajectoryOpParamField> ExternalAxisSearchOp::paramFields() const
{
	return {
		boolParamField("allowCoupledRefine", "Coupled refine", "联立微调", true, 0),
	};
}

bool ExternalAxisSearchOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string ExternalAxisSearchOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op,
												const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool ExternalAxisSearchOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
									   RobotInstruction::UnifiedTrajectory& traj,
									   const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	TrajectoryOpExecutionContext local = ctx;
	local.externalAxisAllowCoupledRefine = trajectoryParamBool(op.params, "allowCoupledRefine", true);
	externalAxisSearchUnified(traj, local);
	return true;
}

} // namespace trajectory_algo
