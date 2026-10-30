#ifndef TRAJECTORYALGORITHMBUILTINS_SHIFTSTARTOP_H
#define TRAJECTORYALGORITHMBUILTINS_SHIFTSTARTOP_H

/// @file ShiftStartOp.h
/// @brief ShiftStartOp 接口

// ShiftStart：闭合路径旋转起点
#include "ITrajectoryOp.h"

namespace trajectory_algo
{
class ShiftStartOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* kindToken() const override { return "ShiftStart"; }
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

#endif // TRAJECTORYALGORITHMBUILTINS_SHIFTSTARTOP_H
