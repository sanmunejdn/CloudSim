// 从 resource/trajectory 加载并解析参数 schema JSON
#pragma once

#include "TrajectoryOpParamSchema.h"
#include "trajectory_algorithm_global.h"

#include <json.hpp>

#include <optional>
#include <string>
#include <vector>

namespace trajectory_algo
{

TRAJECTORY_ALGORITHM_API std::optional<nlohmann::json> loadTrajectoryJsonFile(
	const std::string& resourceBaseDir,
	const std::string& relativePath);

TRAJECTORY_ALGORITHM_API std::vector<TrajectoryOpParamField> parseSchemaFieldsFromJson(
	const nlohmann::json& schemaRoot);

TRAJECTORY_ALGORITHM_API std::vector<TrajectoryOpParamField> loadCommonScopeFieldsFromJson(
	const std::string& resourceBaseDir);

} // namespace trajectory_algo
