#ifndef TRAJECTORYALGORITHMBUILTINS_TRANSLATEOP_H
#define TRAJECTORYALGORITHMBUILTINS_TRANSLATEOP_H

/// @file TranslateOp.h
/// @brief TranslateOp 接口

// Translate 原子块：程序路点位姿平移
#include "ITrajectoryOp.h"

namespace trajectory_algo
{
class TranslateOp final : public ITrajectoryOp
{
public:
	RobotInstruction::TrajectoryOpKind kind() const override;
	const char* kindToken() const override { return "Translate"; }
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

#endif // TRAJECTORYALGORITHMBUILTINS_TRANSLATEOP_H
