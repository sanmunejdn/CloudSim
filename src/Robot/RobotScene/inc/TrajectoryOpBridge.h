#pragma once

#include "robot_scene_global.h"
#include "TrajectoryPipelineTypes.h"

#include <json.hpp>

#include <string>
#include <vector>

namespace trajectory_algo
{
class ITrajectoryOp;
class TrajectoryOpRegistry;
struct TrajectoryOpParamField;
struct TrajectoryParamValue;
}

namespace RobotInstruction
{

/// 轨迹算法统一入口：实现驻留 RobotScene，UI 经此访问避免重复链接 TrajectoryAlgorithm
ROBOT_SCENE_API trajectory_algo::TrajectoryOpRegistry& trajectoryOpRegistry();
ROBOT_SCENE_API void ensureTrajectoryOpBuiltinsRegistered();
ROBOT_SCENE_API const trajectory_algo::ITrajectoryOp* trajectoryOpGet(TrajectoryOpKind kind);
ROBOT_SCENE_API std::vector<TrajectoryOpKind> trajectoryOpPaletteKinds();

ROBOT_SCENE_API std::vector<trajectory_algo::TrajectoryOpParamField> trajectoryOpAllParamFields(
	const trajectory_algo::ITrajectoryOp& op);
ROBOT_SCENE_API bool trajectoryOpParamRead(
	const TrajectoryOpDescriptor& op,
	const trajectory_algo::TrajectoryOpParamField& field,
	trajectory_algo::TrajectoryParamValue& out);
ROBOT_SCENE_API bool trajectoryOpParamWrite(
	TrajectoryOpDescriptor& op,
	const trajectory_algo::TrajectoryOpParamField& field,
	const trajectory_algo::TrajectoryParamValue& value);

ROBOT_SCENE_API nlohmann::json trajectoryPipelineToJson(
	const std::vector<TrajectoryOpDescriptor>& ops);
ROBOT_SCENE_API bool trajectoryPipelineFromJson(
	const nlohmann::json& j,
	std::vector<TrajectoryOpDescriptor>& out,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction
