#ifndef TRAJECTORYALGORITHMBUILTINS_DELETEOP_H
#define TRAJECTORYALGORITHMBUILTINS_DELETEOP_H

/// @file DeleteOp.h
/// @brief DeleteOp 接口

// Delete 原子块：删除 scope 内路点
#include "ITrajectoryOp.h"

namespace trajectory_algo
{
class DeleteOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* kindToken() const override { return "Delete"; }
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

#endif // TRAJECTORYALGORITHMBUILTINS_DELETEOP_H
