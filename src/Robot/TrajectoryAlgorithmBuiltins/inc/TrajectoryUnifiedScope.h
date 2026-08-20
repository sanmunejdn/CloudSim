#ifndef TRAJECTORYALGORITHMBUILTINS_TRAJECTORYUNIFIEDSCOPE_H
#define TRAJECTORYALGORITHMBUILTINS_TRAJECTORYUNIFIEDSCOPE_H

/// @file TrajectoryUnifiedScope.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryUnifiedScope 接口

// 管道执行期 scope 解析
#include "RobotProgramCatalog.h"
#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <cstddef>
#include <unordered_set>
#include <vector>

namespace trajectory_algo
{
std::unordered_set<std::string> resolveScopeInstructionIds(const RobotInstruction::OpScope& scope,
														   const RobotInstruction::UnifiedTrajectory& traj,
														   const RobotInstruction::RobotProgram* program);

std::vector<std::size_t> resolveScopedPointIndices(const RobotInstruction::UnifiedTrajectory& traj,
												   const RobotInstruction::OpScope& scope,
												   const RobotInstruction::RobotProgram* program);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_TRAJECTORYUNIFIEDSCOPE_H
