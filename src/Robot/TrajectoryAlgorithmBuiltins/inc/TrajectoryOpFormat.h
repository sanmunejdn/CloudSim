#ifndef TRAJECTORYALGORITHMBUILTINS_TRAJECTORYOPFORMAT_H
#define TRAJECTORYALGORITHMBUILTINS_TRAJECTORYOPFORMAT_H

/// @file TrajectoryOpFormat.h
/// @brief TrajectoryOpFormat 接口

// 轨迹块参数在 JSON/UI 中的字段格式化
#include "TrajectoryPipelineTypes.h"

#include <string>

namespace trajectory_algo
{
std::string frameLabel(RobotInstruction::TransformReferenceFrame frame, bool chinese);
std::string scopeKindLabel(RobotInstruction::OpScope::Kind kind, bool chinese);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_TRAJECTORYOPFORMAT_H
