#pragma once

#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <json.hpp>

namespace trajectory_algo
{

double trajectoryParamDouble(const nlohmann::json& params, const char* key, double defaultValue);
int trajectoryParamInt(const nlohmann::json& params, const char* key, int defaultValue);
bool trajectoryParamBool(const nlohmann::json& params, const char* key, bool defaultValue);
std::string trajectoryParamString(const nlohmann::json& params, const char* key, const std::string& defaultValue);

void setTrajectoryParamDouble(nlohmann::json& params, const char* key, double value);
void setTrajectoryParamInt(nlohmann::json& params, const char* key, int value);
void setTrajectoryParamBool(nlohmann::json& params, const char* key, bool value);
void setTrajectoryParamString(nlohmann::json& params, const char* key, const std::string& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::TranslateParams parseTranslateParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeTranslateParams(
	nlohmann::json& params,
	const RobotInstruction::TranslateParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::RotateParams parseRotateParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeRotateParams(
	nlohmann::json& params,
	const RobotInstruction::RotateParams& value);

TRAJECTORY_ALGORITHM_API int parseMirrorAxis(const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeMirrorAxis(nlohmann::json& params, int axis);

TRAJECTORY_ALGORITHM_API int parseDuplicateCount(const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeDuplicateCount(nlohmann::json& params, int count);

TRAJECTORY_ALGORITHM_API RobotInstruction::ResampleParams parseResampleParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeResampleParams(
	nlohmann::json& params,
	const RobotInstruction::ResampleParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::PathOffsetParams parsePathOffsetParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writePathOffsetParams(
	nlohmann::json& params,
	const RobotInstruction::PathOffsetParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::WeaveParams parseWeaveParams(const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeWeaveParams(
	nlohmann::json& params,
	const RobotInstruction::WeaveParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::AssignMotionParams parseAssignMotionParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeAssignMotionParams(
	nlohmann::json& params,
	const RobotInstruction::AssignMotionParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::ApproachParams parseApproachParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeApproachParams(
	nlohmann::json& params,
	const RobotInstruction::ApproachParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::RetractParams parseRetractParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeRetractParams(
	nlohmann::json& params,
	const RobotInstruction::RetractParams& value);

TRAJECTORY_ALGORITHM_API RobotInstruction::ProjectToGeometryParams parseProjectParams(
	const nlohmann::json& params);
TRAJECTORY_ALGORITHM_API void writeProjectParams(
	nlohmann::json& params,
	const RobotInstruction::ProjectToGeometryParams& value);

/// 平移/旋转插值后写回 params
TRAJECTORY_ALGORITHM_API void interpolateTransformParamsInPlace(
	RobotInstruction::TrajectoryOpDescriptor& op,
	double t);

TRAJECTORY_ALGORITHM_API void finalizeTransformDefaultParams(
	RobotInstruction::TrajectoryOpDescriptor& op);

} // namespace trajectory_algo
