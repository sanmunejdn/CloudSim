#ifndef TRAJECTORYALGORITHMBUILTINS_EXTERNALAXISSEARCHOP_H
#define TRAJECTORYALGORITHMBUILTINS_EXTERNALAXISSEARCHOP_H

/// @file ExternalAxisSearchOp.h
/// @brief ExternalAxisSearchOp 接口

// ExternalAxisSearch 原子块：搜索外部轴以满足可达性
#include "ITrajectoryOp.h"

namespace trajectory_algo
{
class ExternalAxisSearchOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* kindToken() const override { return "ExternalAxisSearch"; }
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

#endif // TRAJECTORYALGORITHMBUILTINS_EXTERNALAXISSEARCHOP_H
