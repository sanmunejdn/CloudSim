#ifndef TRAJECTORYALGORITHM_TRAJECTORYOPDESCRIPTORCODEC_H
#define TRAJECTORYALGORITHM_TRAJECTORYOPDESCRIPTORCODEC_H

/// @file TrajectoryOpDescriptorCodec.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryOpDescriptorCodec 接口

#include "trajectory_algorithm_global.h"

#include "TrajectoryPipelineTypes.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace trajectory_algo
{
class ITrajectoryOp;

TRAJECTORY_ALGORITHM_API nlohmann::json toJson(const RobotInstruction::TrajectoryOpDescriptor& op);
TRAJECTORY_ALGORITHM_API bool fromJson(const nlohmann::json& j, RobotInstruction::TrajectoryOpDescriptor& out,
									   std::string* errMsg = nullptr);

TRAJECTORY_ALGORITHM_API nlohmann::json
pipelineToJson(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops);
TRAJECTORY_ALGORITHM_API bool pipelineFromJson(const nlohmann::json& j,
											   std::vector<RobotInstruction::TrajectoryOpDescriptor>& out,
											   std::string* errMsg = nullptr);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYOPDESCRIPTORCODEC_H
