#pragma once

#include "robot_scene_global.h"
#include "TrajectoryPipelineEngine.h"
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
ROBOT_SCENE_API bool ensureTrajectoryOpConfigsLoaded(
	const std::string& resourceBaseDir,
	std::string* errMsg = nullptr);
ROBOT_SCENE_API TrajectoryOpDescriptor trajectoryOpDefaultUnified(
	TrajectoryOpKind kind,
	const OpScope& scope);
ROBOT_SCENE_API const trajectory_algo::ITrajectoryOp* trajectoryOpGet(TrajectoryOpKind kind);
ROBOT_SCENE_API std::vector<TrajectoryOpKind> trajectoryOpPaletteKinds();

ROBOT_SCENE_API std::string trajectoryOpKindToString(TrajectoryOpKind kind);
ROBOT_SCENE_API bool trajectoryOpKindFromString(const std::string& token, TrajectoryOpKind& out);

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

ROBOT_SCENE_API bool validateTrajectoryPipeline(
	const std::vector<TrajectoryOpDescriptor>& ops,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API std::string trajectoryOpProjectTargetBackendId(const TrajectoryOpDescriptor& op);
ROBOT_SCENE_API void trajectoryOpSetProjectTargetBackendId(
	TrajectoryOpDescriptor& op,
	const std::string& backendId);

} // namespace RobotInstruction
