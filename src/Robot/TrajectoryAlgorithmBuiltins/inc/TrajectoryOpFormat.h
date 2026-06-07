// 轨迹块参数在 JSON/UI 中的字段格式化
#pragma once

#include "TrajectoryPipelineTypes.h"

#include <string>

namespace trajectory_algo
{

std::string frameLabel(RobotInstruction::TransformReferenceFrame frame, bool chinese);
std::string scopeKindLabel(RobotInstruction::OpScope::Kind kind, bool chinese);

} // namespace trajectory_algo
