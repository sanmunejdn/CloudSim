#ifndef TRAJECTORYALGORITHM_TRAJECTORYPARAMJSONIO_H
#define TRAJECTORYALGORITHM_TRAJECTORYPARAMJSONIO_H

/// @file TrajectoryParamJsonIo.h
/// @brief TrajectoryParamJsonIo 接口

// 从 resource/trajectory 加载并解析参数 schema JSON
#include "trajectory_algorithm_global.h"

#include "TrajectoryOpParamSchema.h"

#include <optional>
#include <string>
#include <vector>

#include <json.hpp>

namespace trajectory_algo
{
TRAJECTORY_ALGORITHM_API std::optional<nlohmann::json> loadTrajectoryJsonFile(const std::string& resourceBaseDir,
																			  const std::string& relativePath);

TRAJECTORY_ALGORITHM_API std::vector<TrajectoryOpParamField>
parseSchemaFieldsFromJson(const nlohmann::json& schemaRoot);

TRAJECTORY_ALGORITHM_API std::vector<TrajectoryOpParamField>
loadCommonScopeFieldsFromJson(const std::string& resourceBaseDir);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYPARAMJSONIO_H
