#pragma once

#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <cstddef>
#include <string>

namespace RobotInstruction
{
struct UnifiedTrajectory;
class RobotProgram;
}

namespace trajectory_algo
{

/// 非刚性配准轨迹纠正：由 RobotScene 注入，Builtins 不依赖 Data/SPARE 实现
class TRAJECTORY_ALGORITHM_API INonRigidTrajectoryWarp
{
public:
	virtual ~INonRigidTrajectoryWarp() = default;

	virtual bool warp(
		RobotInstruction::UnifiedTrajectory& traj,
		const RobotInstruction::NonRigidRegistrationParams& params,
		const RobotInstruction::OpScope& scope,
		const RobotInstruction::RobotProgram* program,
		std::size_t* missCount,
		std::string* errMsg) const = 0;
};

} // namespace trajectory_algo
