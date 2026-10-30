/// @file OrientToPathOp.cpp
/// @brief OrientToPath 轨迹算子

// OrientToPath：按弦切向+世界Z重建姿态
#include "OrientToPathOp.h"

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

RobotInstruction::TrajectoryOpKind OrientToPathOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::OrientToPath;
}

const char* OrientToPathOp::displayName(const bool chinese) const
{
	return chinese ? "按路径定向" : "Orient To Path";
}

TrajectoryOpCapability OrientToPathOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
OrientToPathOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::OrientToPath;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> OrientToPathOp::paramFields() const
{
	return {
		boolParamField("orient.flipToolZ", "Flip tool Z", "翻转工具Z", false, 0),
	};
}

bool OrientToPathOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string OrientToPathOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool OrientToPathOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	const bool flipZ = trajectoryParamBool(op.params, "orient.flipToolZ", false);
	orientToPathUnifiedInScope(traj, op.scope, ctx.program, flipZ);
	return true;
}

} // namespace trajectory_algo
