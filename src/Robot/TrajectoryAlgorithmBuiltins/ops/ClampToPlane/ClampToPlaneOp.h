#ifndef TRAJECTORYALGORITHMBUILTINS_CLAMPTOPLANEOP_H
#define TRAJECTORYALGORITHMBUILTINS_CLAMPTOPLANEOP_H

/// @file ClampToPlaneOp.h
/// @brief ClampToPlaneOp 接口

// ClampToPlane：将坐标轴锁到给定值
#include "ITrajectoryOp.h"

namespace trajectory_algo
{
class ClampToPlaneOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* kindToken() const override { return "ClampToPlane"; }
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

#endif // TRAJECTORYALGORITHMBUILTINS_CLAMPTOPLANEOP_H
