#ifndef TRAJECTORYALGORITHMBUILTINS_ORIENTTOPATHOP_H
#define TRAJECTORYALGORITHMBUILTINS_ORIENTTOPATHOP_H

/// @file OrientToPathOp.h
/// @brief OrientToPathOp 接口

// OrientToPath：按弦切向+世界Z重建姿态
#include "ITrajectoryOp.h"

namespace trajectory_algo
{
class OrientToPathOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* kindToken() const override { return "OrientToPath"; }
	const char* displayName(bool chinese) const override;
	TrajectoryOpCapability capabilities() const override;
	RobotInstruction::TrajectoryOpDescriptor
	makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const override;
	std::vector<TrajectoryOpParamField> paramFields() const override;
	bool validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const override;
	std::string formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, bool chinese) const override;
	bool processPath(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
					 const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const override;
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_ORIENTTOPATHOP_H
