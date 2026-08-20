#ifndef TRAJECTORYALGORITHM_INONRIGIDTRAJECTORYWARP_H
#define TRAJECTORYALGORITHM_INONRIGIDTRAJECTORYWARP_H

/// @file INonRigidTrajectoryWarp.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 非刚性配准轨迹纠正：由 RobotScene 注入，Builtins 不依赖 Data/SPARE 实现

#include "trajectory_algorithm_global.h"

#include "TrajectoryPipelineTypes.h"

#include <cstddef>
#include <string>

namespace RobotInstruction
{
struct UnifiedTrajectory;
class RobotProgram;
} // namespace RobotInstruction

namespace trajectory_algo
{
/// 非刚性配准轨迹纠正：由 RobotScene 注入，Builtins 不依赖 Data/SPARE 实现
class TRAJECTORY_ALGORITHM_API INonRigidTrajectoryWarp
{
public:
	virtual ~INonRigidTrajectoryWarp() = default;

	virtual bool warp(RobotInstruction::UnifiedTrajectory& traj,
					  const RobotInstruction::NonRigidRegistrationParams& params,
					  const RobotInstruction::OpScope& scope, const RobotInstruction::RobotProgram* program,
					  std::size_t* missCount, std::string* errMsg) const = 0;
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_INONRIGIDTRAJECTORYWARP_H
