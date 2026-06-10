// Retract 原子块：在路径尾端插入退刀点
#pragma once

#include "ITrajectoryOp.h"

namespace trajectory_algo
{

class RetractOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* displayName(bool chinese) const override;
	TrajectoryOpCapability capabilities() const override;
	RobotInstruction::TrajectoryOpDescriptor makeDefaultDescriptor(
		const RobotInstruction::OpScope& defaultScope) const override;
	std::vector<TrajectoryOpParamField> paramFields() const override;
	bool validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const override;
	std::string formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, bool chinese) const override;
	bool processPath(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		RobotInstruction::UnifiedTrajectory& traj,
		const TrajectoryOpExecutionContext& ctx,
		std::string* errMsg) const override;
};

} // namespace trajectory_algo
