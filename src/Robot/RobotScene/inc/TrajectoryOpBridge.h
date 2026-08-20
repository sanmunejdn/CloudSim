#ifndef ROBOTSCENE_TRAJECTORYOPBRIDGE_H
#define ROBOTSCENE_TRAJECTORYOPBRIDGE_H

/// @file TrajectoryOpBridge.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轨迹算法统一入口：实现驻留 RobotScene，UI 经此访问避免重复链接 TrajectoryAlgorithm

#include "robot_scene_global.h"

#include "TrajectoryPipelineEngine.h"
#include "TrajectoryPipelineTypes.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace trajectory_algo
{
class ITrajectoryOp;
class TrajectoryOpRegistry;
struct TrajectoryOpParamField;
struct TrajectoryParamValue;
} // namespace trajectory_algo

namespace RobotInstruction
{
/// 轨迹算法统一入口：实现驻留 RobotScene，UI 经此访问避免重复链接 TrajectoryAlgorithm
ROBOT_SCENE_API trajectory_algo::TrajectoryOpRegistry& trajectoryOpRegistry();
ROBOT_SCENE_API void ensureTrajectoryOpBuiltinsRegistered();
ROBOT_SCENE_API bool ensureTrajectoryOpConfigsLoaded(const std::string& resourceBaseDir, std::string* errMsg = nullptr);
ROBOT_SCENE_API TrajectoryOpDescriptor trajectoryOpDefaultUnified(TrajectoryOpKind kind, const OpScope& scope);
ROBOT_SCENE_API const trajectory_algo::ITrajectoryOp* trajectoryOpGet(TrajectoryOpKind kind);
ROBOT_SCENE_API std::vector<TrajectoryOpKind> trajectoryOpPaletteKinds();

ROBOT_SCENE_API std::string trajectoryOpKindToString(TrajectoryOpKind kind);
ROBOT_SCENE_API bool trajectoryOpKindFromString(const std::string& token, TrajectoryOpKind& out);

ROBOT_SCENE_API std::vector<trajectory_algo::TrajectoryOpParamField>
trajectoryOpAllParamFields(const trajectory_algo::ITrajectoryOp& op);
ROBOT_SCENE_API bool trajectoryOpParamRead(const TrajectoryOpDescriptor& op,
										   const trajectory_algo::TrajectoryOpParamField& field,
										   trajectory_algo::TrajectoryParamValue& out);
ROBOT_SCENE_API bool trajectoryOpParamWrite(TrajectoryOpDescriptor& op,
											const trajectory_algo::TrajectoryOpParamField& field,
											const trajectory_algo::TrajectoryParamValue& value);

ROBOT_SCENE_API nlohmann::json trajectoryPipelineToJson(const std::vector<TrajectoryOpDescriptor>& ops);
ROBOT_SCENE_API bool trajectoryPipelineFromJson(const nlohmann::json& j, std::vector<TrajectoryOpDescriptor>& out,
												std::string* errMsg = nullptr);

ROBOT_SCENE_API bool validateTrajectoryPipeline(const std::vector<TrajectoryOpDescriptor>& ops,
												std::string* errMsg = nullptr);

ROBOT_SCENE_API std::string trajectoryOpProjectTargetBackendId(const TrajectoryOpDescriptor& op);
ROBOT_SCENE_API void trajectoryOpSetProjectTargetBackendId(TrajectoryOpDescriptor& op, const std::string& backendId);

ROBOT_SCENE_API std::string trajectoryOpNonRigidSourceBackendId(const TrajectoryOpDescriptor& op);
ROBOT_SCENE_API std::string trajectoryOpNonRigidTargetBackendId(const TrajectoryOpDescriptor& op);
ROBOT_SCENE_API void trajectoryOpSetNonRigidSourceBackendId(TrajectoryOpDescriptor& op, const std::string& backendId);
ROBOT_SCENE_API void trajectoryOpSetNonRigidTargetBackendId(TrajectoryOpDescriptor& op, const std::string& backendId);

ROBOT_SCENE_API std::string trajectoryOpToWorkpieceExternalTcpBackendId(const TrajectoryOpDescriptor& op);
ROBOT_SCENE_API void trajectoryOpSetToWorkpieceExternalTcpBackendId(TrajectoryOpDescriptor& op,
																	const std::string& backendId);

} // namespace RobotInstruction

#endif // ROBOTSCENE_TRAJECTORYOPBRIDGE_H
